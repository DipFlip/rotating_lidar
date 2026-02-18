/**
 * @brief Point cloud rotator node for rotating LiDAR.
 *
 * Subscribes to /velodyne_points_raw and applies a rotation transform (Rx)
 * based on the LiDAR angle evaluated from a least-squares linear fit.
 *
 * Angle source: /joint_states (JointState with header.stamp) published by
 * the motor controller at ~50Hz. A sliding window of timestamped angle
 * samples is maintained. A linear fit (angle = intercept + slope*(t - t_ref))
 * is recomputed on each new sample. The cloud header timestamp is evaluated
 * against this fit to get the rotation angle. This is robust to jitter in
 * individual angle samples since ~250 points contribute to the fit.
 * After the initial fit, samples with large residuals are flagged and
 * excluded from a second pass (robust regression), eliminating the
 * effect of USB serial jitter on the slope estimate.
 *
 * The motor controller publishes a continuous (monotonically increasing)
 * angle, so no phase-unwrapping is needed.
 *
 * Operates directly on the PointCloud2 byte buffer for minimal overhead.
 * No PCL dependency.
 */

#include <cmath>
#include <cstring>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointcloudRotatorNode : public rclcpp::Node
{
public:
  PointcloudRotatorNode()
  : Node("pointcloud_rotator_node")
  {
    this->declare_parameter<std::string>("joint_name", "plank_to_lidar");
    this->declare_parameter<double>("fit_window_sec", 5.0);
    this->declare_parameter<double>("max_stale_ms", 100.0);
    this->declare_parameter<double>("rotation_axis_y", 0.0);
    this->declare_parameter<double>("rotation_axis_z", -0.01);
    this->declare_parameter<double>("yaw_correction_deg", 0.0);

    joint_name_ = this->get_parameter("joint_name").as_string();
    fit_window_sec_ = this->get_parameter("fit_window_sec").as_double();
    max_stale_ms_ = this->get_parameter("max_stale_ms").as_double();
    rot_axis_y_ = this->get_parameter("rotation_axis_y").as_double();
    rot_axis_z_ = this->get_parameter("rotation_axis_z").as_double();
    double yaw_deg = this->get_parameter("yaw_correction_deg").as_double();
    yaw_cos_ = std::cos(yaw_deg * M_PI / 180.0);
    yaw_sin_ = std::sin(yaw_deg * M_PI / 180.0);

    // Subscriber for timestamped LiDAR angle via JointState
    angle_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == joint_name_ && i < msg->position.size()) {
            double angle = msg->position[i];
            double stamp = rclcpp::Time(msg->header.stamp).seconds();

            std::lock_guard<std::mutex> lock(mutex_);

            // Motor controller publishes continuous (unwrapped) angle,
            // so no phase-unwrapping needed here
            angle_buffer_.push_back({stamp, angle});

            // Trim buffer to fit window
            double cutoff = stamp - fit_window_sec_;
            while (angle_buffer_.size() > 2 && angle_buffer_.front().timestamp < cutoff) {
              angle_buffer_.pop_front();
            }

            update_fit();
            break;
          }
        }
      });

    // Subscriber for raw point cloud
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points_raw", rclcpp::SensorDataQoS(),
      std::bind(&PointcloudRotatorNode::cloud_callback, this, std::placeholders::_1));

    // Publisher for rotated point cloud
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(),
      "Pointcloud rotator initialized (linear-fit, window=%.1fs, max_stale=%.0fms, "
      "rot_axis_y=%.4f, rot_axis_z=%.4f, yaw_correction=%.2f deg)",
      fit_window_sec_, max_stale_ms_, rot_axis_y_, rot_axis_z_, yaw_deg);
  }

private:
  struct AngleSample {
    double timestamp;
    double angle;  // radians, unwrapped
  };

  struct LinearFit {
    double t_ref = 0.0;       // reference time (mean of window) for numerical stability
    double intercept = 0.0;   // angle at t_ref
    double slope = 0.0;       // rad/s
    double latest_stamp = 0.0;
    size_t n_samples = 0;
    bool valid = false;
  };

  // Max residual (radians) before a sample is considered an outlier.
  // At ~236 deg/s = 4.12 rad/s, a 10ms timestamp jitter gives ~0.04 rad residual.
  // Use 0.1 rad (~5.7 deg) as a generous threshold.
  static constexpr double OUTLIER_THRESHOLD_RAD = 0.1;

  struct FitSums {
    double sum_dt = 0, sum_a = 0, sum_dt2 = 0, sum_dta = 0;
    double t_mean = 0;
    size_t n = 0;
  };

  static bool compute_fit_from_sums(const FitSums & s, LinearFit & out)
  {
    if (s.n < 2) return false;
    double denom = s.n * s.sum_dt2 - s.sum_dt * s.sum_dt;
    if (std::abs(denom) < 1e-18) return false;
    out.slope = (s.n * s.sum_dta - s.sum_dt * s.sum_a) / denom;
    out.intercept = (s.sum_a - out.slope * s.sum_dt) / s.n;
    out.t_ref = s.t_mean;
    out.n_samples = s.n;
    out.valid = true;
    return true;
  }

  void update_fit()
  {
    size_t n = angle_buffer_.size();
    if (n < 2) { fit_.valid = false; return; }

    // Pass 1: fit all samples
    double t_sum = 0.0;
    for (const auto & s : angle_buffer_) t_sum += s.timestamp;
    double t_mean = t_sum / n;

    FitSums all;
    all.t_mean = t_mean;
    all.n = n;
    for (const auto & s : angle_buffer_) {
      double dt = s.timestamp - t_mean;
      all.sum_dt  += dt;
      all.sum_a   += s.angle;
      all.sum_dt2 += dt * dt;
      all.sum_dta += dt * s.angle;
    }

    LinearFit initial;
    if (!compute_fit_from_sums(all, initial)) { fit_.valid = false; return; }

    // Pass 2: refit excluding outliers (samples with large residuals)
    FitSums clean;
    clean.t_mean = t_mean;
    size_t outliers = 0;
    for (const auto & s : angle_buffer_) {
      double dt = s.timestamp - t_mean;
      double predicted = initial.intercept + initial.slope * dt;
      double residual = std::abs(s.angle - predicted);
      if (residual > OUTLIER_THRESHOLD_RAD) {
        outliers++;
        continue;
      }
      clean.sum_dt  += dt;
      clean.sum_a   += s.angle;
      clean.sum_dt2 += dt * dt;
      clean.sum_dta += dt * s.angle;
      clean.n++;
    }

    if (clean.n >= 2 && compute_fit_from_sums(clean, fit_)) {
      fit_.latest_stamp = angle_buffer_.back().timestamp;
      if (outliers > 0) {
        RCLCPP_DEBUG(this->get_logger(),
          "Fit: rejected %zu/%zu outliers, slope=%.1f deg/s",
          outliers, n, fit_.slope * 180.0 / M_PI);
      }
    } else {
      // Fallback to initial fit if too many outliers
      fit_ = initial;
      fit_.latest_stamp = angle_buffer_.back().timestamp;
    }
  }

  double evaluate_fit(double t) const
  {
    return fit_.intercept + fit_.slope * (t - fit_.t_ref);
  }

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!fit_.valid) {
      // Don't publish until we have a valid angle fit (i.e. calibration done)
      return;
    }

    // Find field offsets
    int x_offset = -1, y_offset = -1, z_offset = -1, time_offset = -1;
    for (const auto & field : msg->fields) {
      if (field.name == "x") x_offset = field.offset;
      else if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
      else if (field.name == "time") time_offset = field.offset;
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "PointCloud2 missing x/y/z fields");
      cloud_pub_->publish(*msg);
      return;
    }

    double cloud_stamp = rclcpp::Time(msg->header.stamp).seconds();
    double stale_ms = (cloud_stamp - fit_.latest_stamp) * 1000.0;

    // Drop if angle data is too stale
    if (stale_ms > max_stale_ms_) {
      RCLCPP_WARN(this->get_logger(),
        "Dropping cloud: angle data %.1fms stale (threshold %.0fms, fit_n=%zu)",
        stale_ms, max_stale_ms_, fit_.n_samples);
      dropped_count_++;
      return;
    }

    double angle = evaluate_fit(cloud_stamp);

    published_count_++;
    if ((published_count_ + dropped_count_) % 500 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Stats: published=%lu dropped=%lu (%.1f%% drop), fit: n=%zu slope=%.1f deg/s stale=%.1fms",
        published_count_, dropped_count_,
        dropped_count_ > 0 ? 100.0 * dropped_count_ / (published_count_ + dropped_count_) : 0.0,
        fit_.n_samples, fit_.slope * 180.0 / M_PI, stale_ms);
    }

    float cos_f = static_cast<float>(std::cos(angle));
    float sin_f = static_cast<float>(std::sin(angle));
    float ay = static_cast<float>(rot_axis_y_);
    float az = static_cast<float>(rot_axis_z_);

    auto output = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    const uint32_t point_step = output->point_step;
    const uint32_t num_points = output->width * output->height;
    uint8_t * data = output->data.data();

    float yc = static_cast<float>(yaw_cos_);
    float ys = static_cast<float>(yaw_sin_);

    for (uint32_t i = 0; i < num_points; ++i) {
      uint8_t * point_ptr = data + i * point_step;

      float x, y, z;
      std::memcpy(&x, point_ptr + x_offset, sizeof(float));
      std::memcpy(&y, point_ptr + y_offset, sizeof(float));
      std::memcpy(&z, point_ptr + z_offset, sizeof(float));

      // Step 1: Rz yaw correction in sensor frame (corrects mounting yaw)
      float x_yc = x * yc - y * ys;
      float y_yc = x * ys + y * yc;

      // Step 2: Rx rotation around the physical shaft axis
      float dy = y_yc - ay;
      float dz = z - az;
      float y_new = dy * cos_f - dz * sin_f + ay;
      float z_new = dy * sin_f + dz * cos_f + az;

      std::memcpy(point_ptr + x_offset, &x_yc, sizeof(float));
      std::memcpy(point_ptr + y_offset, &y_new, sizeof(float));
      std::memcpy(point_ptr + z_offset, &z_new, sizeof(float));

      // Zero per-point timestamps to avoid negative values that
      // cause issues in Cartographer.
      if (time_offset >= 0) {
        float zero = 0.0f;
        std::memcpy(point_ptr + time_offset, &zero, sizeof(float));
      }
    }

    cloud_pub_->publish(*output);
  }

  std::string joint_name_;
  double fit_window_sec_;
  double max_stale_ms_;
  double rot_axis_y_;
  double rot_axis_z_;
  double yaw_cos_;
  double yaw_sin_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::mutex mutex_;
  std::deque<AngleSample> angle_buffer_;
  LinearFit fit_;
  uint64_t published_count_ = 0;
  uint64_t dropped_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudRotatorNode>());
  rclcpp::shutdown();
  return 0;
}
