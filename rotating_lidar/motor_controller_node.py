#!/usr/bin/env python3
"""Feetech STS3215 motor controller node for rotating LiDAR.

Controls the servo motor that rotates the VLP-16 LiDAR around a horizontal axis.
Reads motor position register at ~50Hz, computes LiDAR angle via gear ratio,
and publishes both raw motor angle and derived LiDAR angle.

Calibration: On start_motor, collects 10 hall sensor triggers and records the
cumulative motor angle at each. The residuals (motor_angle mod motor_period)
are averaged to find the "golden zero" — the motor position within one lidar
revolution where the hall fires. From the golden zero and the hall_offset_deg
parameter, horizontal (lidar_angle=0) is computed. The reference is updated on
every subsequent hall trigger to prevent drift.
"""

import math
import signal
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Header
from std_srvs.srv import Trigger, SetBool

try:
    from scservo_sdk import (
        PortHandler,
        PacketHandler,
        COMM_SUCCESS,
    )
except ImportError:
    PortHandler = None
    PacketHandler = None
    COMM_SUCCESS = 0

# STS3215 control table addresses
ADDR_SCS_TORQUE_ENABLE = 40
ADDR_SCS_GOAL_POSITION = 42
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_PRESENT_POSITION = 56
ADDR_SCS_MODE = 33

SCS_PROTOCOL_VERSION = 0
MODE_POSITION = 0
MODE_WHEEL = 1

POSITION_STEPS = 4096
DEGREES_PER_STEP = 360.0 / POSITION_STEPS

# Calibration constants
CALIBRATION_SAMPLES = 10
HALL_WATCHDOG_THRESHOLD_DEG = 60.0


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/feetech')
        self.declare_parameter('servo_id', 7)
        self.declare_parameter('spin_speed', 2000)
        self.declare_parameter('gear_ratio', 1.3333333333)  # 24 motor teeth / 18 lidar shaft teeth
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('position_read_rate', 50.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('hall_offset_deg', 30.0)  # lidar angle (deg) at hall trigger

        # Get parameters
        self.port = self.get_parameter('port').value
        self.servo_id = self.get_parameter('servo_id').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.baudrate = self.get_parameter('baudrate').value
        self.position_read_rate = self.get_parameter('position_read_rate').value
        self.hall_offset_deg = self.get_parameter('hall_offset_deg').value

        # Derived constant: motor degrees per full lidar revolution
        self.motor_deg_per_lidar_rev = 360.0 / self.gear_ratio

        # Publishers
        self.motor_angle_pub = self.create_publisher(Float64, '/rotating_lidar/motor_angle', 10)
        self.lidar_angle_pub = self.create_publisher(Float64, '/rotating_lidar/lidar_angle', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Services
        self.create_service(Trigger, '~/start_motor', self.start_motor_callback)
        self.create_service(Trigger, '~/stop_motor', self.stop_motor_callback)
        self.create_service(Trigger, '~/calibrate', self.calibrate_callback)
        self.create_service(SetBool, '~/set_speed', self.set_speed_callback)

        # Hall sensor subscription
        self.create_subscription(Header, '/rotating_lidar/hall_trigger', self.hall_trigger_callback, 10)

        # Motor state
        self.motor_running = False
        self.cumulative_angle_deg = 0.0
        self.last_raw_position = None

        # Calibration state
        self.cal_state = 'UNCALIBRATED'
        self.cal_samples = []
        # golden_zero: motor position (mod motor_deg_per_lidar_rev) where hall fires
        self.golden_zero = 0.0
        # hall_motor_ref: cumulative motor angle at last hall trigger
        self.hall_motor_ref = 0.0
        self.last_hall_motor_deg = None  # for debounce

        # Serial
        self.port_handler = None
        self.packet_handler = None
        self._init_serial()

        # Position reading timer
        self.create_timer(1.0 / self.position_read_rate, self.read_position_callback)

        self.get_logger().info(
            f'Motor controller initialized: port={self.port}, id={self.servo_id}, '
            f'gear_ratio={self.gear_ratio}, '
            f'motor_deg_per_lidar_rev={self.motor_deg_per_lidar_rev:.1f}'
        )

        # Auto-start motor if configured
        if self.get_parameter('auto_start').value:
            success, message = self._start_spinning()
            if success:
                self._begin_calibration()
                self.get_logger().info(f'Auto-start: {message}. Calibrating...')
            else:
                self.get_logger().error(f'Auto-start failed: {message}')

    def _init_serial(self):
        """Initialize serial connection to Feetech servo."""
        if PortHandler is None:
            self.get_logger().error('scservo_sdk not installed')
            return

        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(SCS_PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port: {self.port}')
            return

        if not self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().error(f'Failed to set baudrate: {self.baudrate}')
            return

        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_MODE, MODE_WHEEL
        )
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f'Failed to set wheel mode: {self.packet_handler.getTxRxResult(result)}'
            )

        self.get_logger().info('Serial connection established')

    def _start_spinning(self):
        """Enable torque and set speed. Returns (success, message)."""
        if self.port_handler is None or self.packet_handler is None:
            return False, 'Serial connection not initialized'

        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_TORQUE_ENABLE, 1
        )
        if result != COMM_SUCCESS:
            return False, f'Failed to enable torque: {self.packet_handler.getTxRxResult(result)}'

        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_GOAL_SPEED, self.spin_speed
        )
        if result != COMM_SUCCESS:
            return False, f'Failed to set speed: {self.packet_handler.getTxRxResult(result)}'

        self.motor_running = True
        self.cumulative_angle_deg = 0.0
        self.last_raw_position = None
        return True, f'Motor started at speed {self.spin_speed}'

    def _begin_calibration(self):
        """Enter calibration state."""
        self.cal_state = 'CALIBRATING'
        self.cal_samples = []
        self.cal_skip_first = True  # skip first hall trigger (motor still spinning up)
        self.get_logger().info(
            f'Calibration started: skipping first trigger, then collecting '
            f'{CALIBRATION_SAMPLES} hall triggers...'
        )

    def start_motor_callback(self, request, response):
        """Start motor and begin calibration."""
        success, message = self._start_spinning()
        if not success:
            response.success = False
            response.message = message
            return response

        self._begin_calibration()
        response.success = True
        response.message = f'{message}. Calibrating over {CALIBRATION_SAMPLES} triggers...'
        self.get_logger().info(response.message)
        return response

    def calibrate_callback(self, request, response):
        """Manually trigger recalibration."""
        if not self.motor_running:
            response.success = False
            response.message = 'Motor must be running to calibrate'
            return response

        self._begin_calibration()
        response.success = True
        response.message = f'Recalibration started: collecting {CALIBRATION_SAMPLES} hall triggers...'
        return response

    def stop_motor_callback(self, request, response):
        """Stop the motor."""
        if self.port_handler is None or self.packet_handler is None:
            response.success = False
            response.message = 'Serial connection not initialized'
            return response

        self.packet_handler.write2ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_GOAL_SPEED, 0
        )
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_TORQUE_ENABLE, 0
        )

        self.motor_running = False
        response.success = True
        response.message = 'Motor stopped'
        self.get_logger().info(response.message)
        return response

    def set_speed_callback(self, request, response):
        """Toggle motor direction."""
        if request.data:
            speed = self.spin_speed
        else:
            speed = self.spin_speed | 0x0400

        if self.port_handler and self.packet_handler and self.motor_running:
            self.packet_handler.write2ByteTxRx(
                self.port_handler, self.servo_id, ADDR_SCS_GOAL_SPEED, speed
            )
            response.success = True
            response.message = f'Direction set: {"forward" if request.data else "reverse"}'
        else:
            response.success = False
            response.message = 'Motor not running or serial not initialized'
        return response

    def read_position_callback(self):
        """Read motor position and publish angles."""
        if self.port_handler is None or self.packet_handler is None:
            return

        position, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            return

        raw_angle_deg = position * DEGREES_PER_STEP

        # Track cumulative angle (handle wraparound)
        if self.last_raw_position is not None:
            delta = raw_angle_deg - self.last_raw_position
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            self.cumulative_angle_deg += delta
        self.last_raw_position = raw_angle_deg

        # Publish raw motor angle (degrees)
        motor_msg = Float64()
        motor_msg.data = self.cumulative_angle_deg
        self.motor_angle_pub.publish(motor_msg)

        # Compute LiDAR angle relative to calibrated horizontal
        if self.cal_state == 'CALIBRATED':
            # Motor degrees since last hall trigger
            motor_since_hall = self.cumulative_angle_deg - self.hall_motor_ref
            # Convert to lidar degrees within one revolution
            lidar_angle_deg = (motor_since_hall % self.motor_deg_per_lidar_rev) \
                / self.motor_deg_per_lidar_rev * 360.0
            # At the hall trigger (motor_since_hall=0), lidar is at hall_offset_deg
            lidar_angle_deg = lidar_angle_deg + self.hall_offset_deg
            # Normalize to [-180, +180]
            lidar_angle_deg = lidar_angle_deg % 360.0
            if lidar_angle_deg > 180.0:
                lidar_angle_deg -= 360.0
        else:
            lidar_angle_deg = (self.cumulative_angle_deg * self.gear_ratio) % 360.0

        lidar_angle_rad = math.radians(lidar_angle_deg)

        lidar_msg = Float64()
        lidar_msg.data = lidar_angle_rad
        self.lidar_angle_pub.publish(lidar_msg)

        # Publish JointState for robot_state_publisher (drives URDF continuous joint)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['plank_to_lidar']
        js.position = [lidar_angle_rad]
        self.joint_state_pub.publish(js)

    def hall_trigger_callback(self, msg):
        """Handle hall sensor trigger."""
        if self.last_raw_position is None:
            return

        motor_deg = self.cumulative_angle_deg

        # Debounce: ignore triggers less than half a lidar revolution apart
        min_motor_spacing = self.motor_deg_per_lidar_rev * 0.5
        if self.last_hall_motor_deg is not None:
            if motor_deg - self.last_hall_motor_deg < min_motor_spacing:
                self.get_logger().debug(
                    f'Hall debounce: ignoring trigger at {motor_deg:.1f} deg '
                    f'(only {motor_deg - self.last_hall_motor_deg:.1f} deg from last)'
                )
                return
        self.last_hall_motor_deg = motor_deg

        # Motor position within one lidar revolution
        residual = motor_deg % self.motor_deg_per_lidar_rev

        if self.cal_state == 'CALIBRATING':
            if self.cal_skip_first:
                self.cal_skip_first = False
                self.get_logger().info(
                    f'Calibration: skipped first trigger (motor_angle={motor_deg:.1f})'
                )
                return

            self.cal_samples.append(residual)
            n = len(self.cal_samples)
            self.get_logger().info(
                f'Calibration sample {n}/{CALIBRATION_SAMPLES}: '
                f'motor_angle={motor_deg:.1f}, '
                f'residual={residual:.1f} deg (mod {self.motor_deg_per_lidar_rev:.1f})'
            )

            if n >= CALIBRATION_SAMPLES:
                self._finish_calibration(motor_deg)

        elif self.cal_state == 'CALIBRATED':
            # Update reference on every hall trigger to prevent drift
            self.hall_motor_ref = motor_deg

            # Watchdog: check residual vs golden zero
            error = residual - self.golden_zero
            # Normalize to [-half_period, +half_period]
            half_period = self.motor_deg_per_lidar_rev / 2.0
            if error > half_period:
                error -= self.motor_deg_per_lidar_rev
            elif error < -half_period:
                error += self.motor_deg_per_lidar_rev
            # Convert to lidar degrees
            error_lidar = error * self.gear_ratio

            if abs(error_lidar) > HALL_WATCHDOG_THRESHOLD_DEG:
                self.get_logger().warn(
                    f'Hall watchdog: error={error_lidar:.1f} deg exceeds '
                    f'{HALL_WATCHDOG_THRESHOLD_DEG} deg threshold. Recalibrating...'
                )
                self._begin_calibration()
            else:
                self.get_logger().debug(
                    f'Hall watchdog OK: error={error_lidar:.1f} deg'
                )

    def _finish_calibration(self, last_motor_deg):
        """Compute golden zero from calibration residuals.

        Each residual is motor_angle % motor_deg_per_lidar_rev — the position
        within one lidar revolution where the hall fired. Circular-average them
        to get the golden zero. Log the per-sample deviations from the average.
        """
        period = self.motor_deg_per_lidar_rev

        # Circular mean of residuals (handles wraparound near 0/period)
        sin_sum = 0.0
        cos_sum = 0.0
        for r in self.cal_samples:
            phase = r / period * 2.0 * math.pi
            sin_sum += math.sin(phase)
            cos_sum += math.cos(phase)

        avg_phase = math.atan2(sin_sum, cos_sum)
        if avg_phase < 0:
            avg_phase += 2.0 * math.pi
        self.golden_zero = avg_phase / (2.0 * math.pi) * period

        # Consistency (R = 1.0 means all samples agree perfectly)
        r = math.sqrt(sin_sum**2 + cos_sum**2) / len(self.cal_samples)

        # Log per-sample deviations from the golden zero
        deviations = []
        for res in self.cal_samples:
            dev = res - self.golden_zero
            half = period / 2.0
            if dev > half:
                dev -= period
            elif dev < -half:
                dev += period
            deviations.append(dev)
        dev_str = ', '.join(f'{d:+.1f}' for d in deviations)

        # Set reference to the most recent hall trigger
        self.hall_motor_ref = last_motor_deg

        self.cal_state = 'CALIBRATED'
        self.get_logger().info(
            f'Calibration complete! golden_zero={self.golden_zero:.1f} deg '
            f'(mod {period:.1f}), '
            f'hall_offset_deg={self.hall_offset_deg}, '
            f'consistency={r:.3f}, '
            f'deviations=[{dev_str}]'
        )

    def _return_to_horizontal(self):
        """Keep spinning at normal speed, track lidar angle, stop at horizontal."""
        self.get_logger().info('Returning to horizontal...')

        period = self.motor_deg_per_lidar_rev
        cumulative = self.cumulative_angle_deg
        last_raw = self.last_raw_position

        for _ in range(500):  # 10s at 50Hz
            time.sleep(0.02)
            position, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, self.servo_id, ADDR_SCS_PRESENT_POSITION
            )
            if result != COMM_SUCCESS:
                continue

            # Update cumulative angle (same logic as read_position_callback)
            raw_deg = position * DEGREES_PER_STEP
            if last_raw is not None:
                delta = raw_deg - last_raw
                if delta > 180.0:
                    delta -= 360.0
                elif delta < -180.0:
                    delta += 360.0
                cumulative += delta
            last_raw = raw_deg

            # Compute lidar angle from hall reference
            motor_since_hall = cumulative - self.hall_motor_ref
            lidar_deg = (motor_since_hall % period) / period * 360.0 + self.hall_offset_deg
            lidar_deg = lidar_deg % 360.0

            # Stop when lidar angle is near 0 (horizontal)
            # Check proximity to 0 (i.e. lidar_deg near 0 or near 360)
            if lidar_deg > 180.0:
                lidar_deg -= 360.0
            if abs(lidar_deg) < 15.0:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, self.servo_id, ADDR_SCS_GOAL_SPEED, 0
                )
                self.get_logger().info(
                    f'Reached horizontal position (lidar_angle={lidar_deg:.1f} deg)'
                )
                return

        self.packet_handler.write2ByteTxRx(
            self.port_handler, self.servo_id, ADDR_SCS_GOAL_SPEED, 0
        )
        self.get_logger().warn('Timed out waiting for horizontal position')

    def destroy_node(self):
        """Return to horizontal and clean up serial connection on shutdown."""
        if self.motor_running and self.port_handler and self.packet_handler:
            if self.cal_state == 'CALIBRATED':
                try:
                    self._return_to_horizontal()
                except Exception as e:
                    self.get_logger().warn(f'Failed to return to horizontal: {e}')

            # Disable torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.servo_id, ADDR_SCS_TORQUE_ENABLE, 0
            )

        if self.port_handler:
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()

    shutdown_done = False

    def shutdown():
        nonlocal shutdown_done
        if shutdown_done:
            return
        shutdown_done = True
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def sigterm_handler(signum, frame):
        node.get_logger().info('SIGTERM received, shutting down...')
        shutdown()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, sigterm_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        shutdown()


if __name__ == '__main__':
    main()
