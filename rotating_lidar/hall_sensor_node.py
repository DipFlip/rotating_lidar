#!/usr/bin/env python3
"""Hall sensor node for rotating LiDAR zero-reference detection.

Reads serial output from an ESP32-C3 connected to a hall sensor that detects
each pass of the LiDAR through its reference (horizontal) orientation.
Publishes a trigger message on each magnet pass for motor angle correction.

Expected ESP32 serial format: H,<count>,<millis>

The ESP32's millis timestamp is used to reconstruct accurate ROS timestamps,
avoiding jitter from Python timer polling under CPU load. A clock offset
between ESP32 uptime and ROS time is established on the first message and
updated on each subsequent message to track clock drift.
"""

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class HallSensorNode(Node):
    def __init__(self):
        super().__init__('hall_sensor_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/esp32_hall')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_interval', 2.0)

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        # Publisher
        self.trigger_pub = self.create_publisher(Header, '/rotating_lidar/hall_trigger', 10)

        # State
        self.serial_conn = None
        self.last_count = -1
        self.reconnect_timer = None

        # Clock synchronisation: ros_time = esp32_sec + clock_offset
        self.clock_offset = None

        # Try initial connection
        self._connect_serial()

        # Read timer - check serial at 100Hz
        self.create_timer(0.01, self.read_serial_callback)

        self.get_logger().info(f'Hall sensor node initialized: port={self.port}')

    def _connect_serial(self):
        """Attempt to open serial connection."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.01,
            )
            # ESP32-C3 resets on serial open (DTR). Wait for boot and
            # flush ROM output so we only parse firmware messages.
            import time
            time.sleep(1.0)
            self.serial_conn.reset_input_buffer()
            self.get_logger().info(f'Serial connected: {self.port}')
            # Reset clock sync on reconnect
            self.clock_offset = None
            # Cancel reconnect timer if active
            if self.reconnect_timer is not None:
                self.reconnect_timer.cancel()
                self.reconnect_timer = None
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial connection failed: {e}')
            self.serial_conn = None
            # Schedule reconnection
            if self.reconnect_timer is None:
                self.reconnect_timer = self.create_timer(
                    self.reconnect_interval, self._reconnect_callback
                )

    def _reconnect_callback(self):
        """Periodically attempt to reconnect serial."""
        if self.serial_conn is None:
            self._connect_serial()

    def _esp32_to_ros_time(self, esp32_millis):
        """Convert ESP32 millis to ROS time using tracked clock offset.

        On first call, establishes the offset. On subsequent calls, uses
        a simple low-pass update so the offset tracks any long-term drift
        between the ESP32 crystal and system clock without being affected
        by single-message jitter.
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        esp32_sec = esp32_millis * 0.001

        if self.clock_offset is None:
            self.clock_offset = now_sec - esp32_sec
        else:
            # Slowly track drift (alpha=0.01 → time constant ~100 messages)
            new_offset = now_sec - esp32_sec
            self.clock_offset += 0.01 * (new_offset - self.clock_offset)

        return esp32_sec + self.clock_offset

    def read_serial_callback(self):
        """Read and parse serial data from ESP32."""
        if self.serial_conn is None:
            return

        try:
            if self.serial_conn.in_waiting == 0:
                return

            line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            # Parse format: H,<count>,<millis>
            if not line.startswith('H,'):
                return
            parts = line.split(',')
            if len(parts) < 3:
                return
            try:
                count = int(parts[1])
                esp32_millis = int(parts[2])
            except ValueError:
                return

            if count != self.last_count:
                self.last_count = count
                trigger_sec = self._esp32_to_ros_time(esp32_millis)
                sec = int(trigger_sec)
                nanosec = int((trigger_sec - sec) * 1e9)

                msg = Header()
                msg.stamp.sec = sec
                msg.stamp.nanosec = nanosec
                msg.frame_id = 'hall_sensor'
                self.trigger_pub.publish(msg)
                self.get_logger().debug(f'Hall trigger: count={count}')

        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            self.serial_conn = None
            # Schedule reconnection
            if self.reconnect_timer is None:
                self.reconnect_timer = self.create_timer(
                    self.reconnect_interval, self._reconnect_callback
                )

    def destroy_node(self):
        """Clean up serial connection."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HallSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
