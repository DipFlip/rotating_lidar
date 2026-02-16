/**
 * @brief Point cloud rotator node for rotating LiDAR.
 *
 * Subscribes to /velodyne_points_raw and applies a rotation transform (Rx)
 * based on the current LiDAR angle, then publishes on /velodyne_points.
 *
 * Uses the per-point 'time' field from the VLP-16 to interpolate the rotation
 * angle for each point, accounting for motor rotation during the ~100ms scan.
 *
 * Operates directly on the PointCloud2 byte buffer for minimal overhead.
 * No PCL dependency - pure in-place buffer manipulation.
 */

#include <cmath>
#include <cstring>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

class PointcloudRotatorNode : public rclcpp::Node
{
public:
  PointcloudRotatorNode()
  : Node("pointcloud_rotator_node"),
    current_angle_(0.0),
    prev_angle_(0.0),
    angular_velocity_(0.0),
    last_angle_time_(0, 0, RCL_ROS_TIME),
    angle_received_(false),
    velocity_valid_(false)
  {
    // Subscriber for LiDAR angle (radians)
    angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/rotating_lidar/lidar_angle", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(angle_mutex_);
        auto now = this->get_clock()->now();

        if (angle_received_) {
          double dt = (now - last_angle_time_).seconds();
          if (dt > 0.001) {  // Avoid division by zero
            angular_velocity_ = (msg->data - current_angle_) / dt;
            velocity_valid_ = true;
          }
        }

        prev_angle_ = current_angle_;
        current_angle_ = msg->data;
        last_angle_time_ = now;
        angle_received_ = true;
      });

    // Subscriber for raw point cloud
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points_raw", rclcpp::SensorDataQoS(),
      std::bind(&PointcloudRotatorNode::cloud_callback, this, std::placeholders::_1));

    // Publisher for rotated point cloud
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "Pointcloud rotator node initialized (per-point interpolation)");
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    double base_angle;
    double ang_vel;
    bool do_interpolate;
    {
      std::lock_guard<std::mutex> lock(angle_mutex_);
      if (!angle_received_) {
        // No angle received yet, pass through unmodified
        cloud_pub_->publish(*msg);
        return;
      }
      base_angle = current_angle_;
      ang_vel = angular_velocity_;
      do_interpolate = velocity_valid_;
    }

    // Find field offsets
    int y_offset = -1, z_offset = -1, time_offset = -1;
    for (const auto & field : msg->fields) {
      if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
      else if (field.name == "time") time_offset = field.offset;
    }

    if (y_offset < 0 || z_offset < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "PointCloud2 missing y/z fields");
      cloud_pub_->publish(*msg);
      return;
    }

    // Create output message
    auto output = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);

    const uint32_t point_step = output->point_step;
    const uint32_t num_points = output->width * output->height;
    uint8_t * data = output->data.data();

    // If we have per-point timestamps and angular velocity, interpolate per-point.
    // The VLP-16 'time' field gives seconds relative to the scan (negative = earlier).
    // The last point has time ~0, the first has time ~-0.1s.
    // Per-point angle = base_angle + angular_velocity * point_time
    if (do_interpolate && time_offset >= 0) {
      for (uint32_t i = 0; i < num_points; ++i) {
        uint8_t * point_ptr = data + i * point_step;

        float point_time;
        std::memcpy(&point_time, point_ptr + time_offset, sizeof(float));

        // Per-point angle: base_angle is the angle at scan end (time=0),
        // point_time is negative for earlier points
        double angle = base_angle + ang_vel * static_cast<double>(point_time);
        float cos_f = static_cast<float>(std::cos(angle));
        float sin_f = static_cast<float>(std::sin(angle));

        float y, z;
        std::memcpy(&y, point_ptr + y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + z_offset, sizeof(float));

        float y_new = y * cos_f - z * sin_f;
        float z_new = y * sin_f + z * cos_f;

        std::memcpy(point_ptr + y_offset, &y_new, sizeof(float));
        std::memcpy(point_ptr + z_offset, &z_new, sizeof(float));
      }
    } else {
      // Fallback: single angle for entire cloud
      float cos_f = static_cast<float>(std::cos(base_angle));
      float sin_f = static_cast<float>(std::sin(base_angle));

      for (uint32_t i = 0; i < num_points; ++i) {
        uint8_t * point_ptr = data + i * point_step;

        float y, z;
        std::memcpy(&y, point_ptr + y_offset, sizeof(float));
        std::memcpy(&z, point_ptr + z_offset, sizeof(float));

        float y_new = y * cos_f - z * sin_f;
        float z_new = y * sin_f + z * cos_f;

        std::memcpy(point_ptr + y_offset, &y_new, sizeof(float));
        std::memcpy(point_ptr + z_offset, &z_new, sizeof(float));
      }
    }

    cloud_pub_->publish(*output);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::mutex angle_mutex_;
  double current_angle_;
  double prev_angle_;
  double angular_velocity_;
  rclcpp::Time last_angle_time_;
  bool angle_received_;
  bool velocity_valid_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudRotatorNode>());
  rclcpp::shutdown();
  return 0;
}
