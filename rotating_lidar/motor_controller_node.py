#!/usr/bin/env python3
"""Feetech STS3215 motor controller node for rotating LiDAR.

Controls the servo motor that rotates the VLP-16 LiDAR around a horizontal axis.
Reads motor position register at ~50Hz, computes LiDAR angle via gear ratio,
and publishes both raw motor angle and derived LiDAR angle.

Calibration: On start_motor, spins for 10 revolutions while recording the motor
position at each hall sensor trigger. The average gives the motor position that
corresponds to LiDAR horizontal (angle=0). After calibration, only the motor
register is used for angle computation. The hall sensor acts as a watchdog —
if it triggers more than 30 degrees from expected horizontal, recalibration runs.
"""

import math

import rclpy
from rclpy.node import Node
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
ADDR_SCS_GOAL_SPEED = 46
ADDR_SCS_PRESENT_POSITION = 56
ADDR_SCS_MODE = 33

SCS_PROTOCOL_VERSION = 0
MODE_WHEEL = 1

POSITION_STEPS = 4096
DEGREES_PER_STEP = 360.0 / POSITION_STEPS

# Calibration constants
CALIBRATION_REVOLUTIONS = 10
HALL_WATCHDOG_THRESHOLD_DEG = 30.0


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/feetech')
        self.declare_parameter('servo_id', 7)
        self.declare_parameter('spin_speed', 2000)
        self.declare_parameter('gear_ratio', 1.3333)  # motor_teeth / lidar_teeth (24/18)
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('position_read_rate', 50.0)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.servo_id = self.get_parameter('servo_id').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.baudrate = self.get_parameter('baudrate').value
        self.position_read_rate = self.get_parameter('position_read_rate').value

        # Publishers
        self.motor_angle_pub = self.create_publisher(Float64, '/rotating_lidar/motor_angle', 10)
        self.lidar_angle_pub = self.create_publisher(Float64, '/rotating_lidar/lidar_angle', 10)

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
        # UNCALIBRATED: no reference, lidar_angle published as raw (no offset)
        # CALIBRATING: collecting hall trigger motor positions
        # CALIBRATED: horizontal_motor_deg is known, used as zero reference
        self.cal_state = 'UNCALIBRATED'
        self.horizontal_motor_deg = 0.0  # Motor position (cumulative deg) at LiDAR horizontal
        self.cal_samples = []  # Motor positions at hall triggers during calibration
        self.cal_start_angle = 0.0  # Motor angle when calibration started

        # Serial
        self.port_handler = None
        self.packet_handler = None
        self._init_serial()

        # Position reading timer
        self.create_timer(1.0 / self.position_read_rate, self.read_position_callback)

        self.get_logger().info(
            f'Motor controller initialized: port={self.port}, id={self.servo_id}, '
            f'gear_ratio={self.gear_ratio}'
        )

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
        self.cal_start_angle = self.cumulative_angle_deg
        self.get_logger().info(
            f'Calibration started: collecting {CALIBRATION_REVOLUTIONS} hall triggers...'
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
        response.message = f'{message}. Calibrating over {CALIBRATION_REVOLUTIONS} revolutions...'
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
        response.message = f'Recalibration started: collecting {CALIBRATION_REVOLUTIONS} hall triggers...'
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
        # lidar_angle = (motor_angle - horizontal_ref) * gear_ratio, in radians
        if self.cal_state == 'CALIBRATED':
            lidar_angle_deg = (self.cumulative_angle_deg - self.horizontal_motor_deg) * self.gear_ratio
        else:
            # Before calibration, publish raw angle (no horizontal reference)
            lidar_angle_deg = self.cumulative_angle_deg * self.gear_ratio

        lidar_angle_rad = math.radians(lidar_angle_deg)

        lidar_msg = Float64()
        lidar_msg.data = lidar_angle_rad
        self.lidar_angle_pub.publish(lidar_msg)

    def hall_trigger_callback(self, msg):
        """Handle hall sensor trigger."""
        if self.last_raw_position is None:
            return

        if self.cal_state == 'CALIBRATING':
            self.cal_samples.append(self.cumulative_angle_deg)
            n = len(self.cal_samples)
            self.get_logger().info(
                f'Calibration sample {n}/{CALIBRATION_REVOLUTIONS}: '
                f'motor_angle={self.cumulative_angle_deg:.1f} deg'
            )

            if n >= CALIBRATION_REVOLUTIONS:
                self._finish_calibration()

        elif self.cal_state == 'CALIBRATED':
            # Watchdog: check if hall trigger is near expected horizontal
            motor_deg_since_horizontal = self.cumulative_angle_deg - self.horizontal_motor_deg
            # Expected: hall fires at multiples of (360 / gear_ratio) motor degrees
            motor_deg_per_lidar_rev = 360.0 / self.gear_ratio
            remainder = motor_deg_since_horizontal % motor_deg_per_lidar_rev
            # Normalize to [-half, +half]
            if remainder > motor_deg_per_lidar_rev / 2:
                remainder -= motor_deg_per_lidar_rev
            error_lidar_deg = remainder * self.gear_ratio

            if abs(error_lidar_deg) > HALL_WATCHDOG_THRESHOLD_DEG:
                self.get_logger().warn(
                    f'Hall watchdog: error={error_lidar_deg:.1f} deg exceeds '
                    f'{HALL_WATCHDOG_THRESHOLD_DEG} deg threshold. Recalibrating...'
                )
                self._begin_calibration()
            else:
                self.get_logger().debug(
                    f'Hall watchdog OK: error={error_lidar_deg:.1f} deg'
                )

    def _finish_calibration(self):
        """Compute horizontal reference from calibration samples."""
        # Each sample is a cumulative motor angle at hall trigger.
        # The spacing between samples should be ~(360/gear_ratio) motor degrees.
        # Average the offset from the first sample modulo one lidar revolution.
        motor_deg_per_lidar_rev = 360.0 / self.gear_ratio

        # Use circular mean to handle wraparound within one motor revolution period
        sin_sum = 0.0
        cos_sum = 0.0
        for sample in self.cal_samples:
            phase = (sample % motor_deg_per_lidar_rev) * math.pi * 2.0 / motor_deg_per_lidar_rev
            sin_sum += math.sin(phase)
            cos_sum += math.cos(phase)

        avg_phase = math.atan2(sin_sum, cos_sum)
        if avg_phase < 0:
            avg_phase += 2.0 * math.pi

        # Convert phase back to motor degrees within one period
        horizontal_within_period = avg_phase * motor_deg_per_lidar_rev / (2.0 * math.pi)

        # Find the closest reference to current position
        current_period = self.cumulative_angle_deg // motor_deg_per_lidar_rev
        self.horizontal_motor_deg = current_period * motor_deg_per_lidar_rev + horizontal_within_period

        self.cal_state = 'CALIBRATED'
        self.get_logger().info(
            f'Calibration complete! horizontal_motor_deg={self.horizontal_motor_deg:.1f}, '
            f'samples={len(self.cal_samples)}, '
            f'motor_deg_per_lidar_rev={motor_deg_per_lidar_rev:.1f}'
        )

    def destroy_node(self):
        """Clean up serial connection on shutdown."""
        if self.motor_running:
            if self.port_handler and self.packet_handler:
                self.packet_handler.write2ByteTxRx(
                    self.port_handler, self.servo_id, ADDR_SCS_GOAL_SPEED, 0
                )
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, self.servo_id, ADDR_SCS_TORQUE_ENABLE, 0
                )
        if self.port_handler:
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
