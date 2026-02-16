#!/usr/bin/env python3
"""Hall sensor node for rotating LiDAR zero-reference detection.

Reads serial output from an ESP32-C3 connected to a hall sensor that detects
each pass of the LiDAR through its reference (horizontal) orientation.
Publishes a trigger message on each magnet pass for motor angle correction.

Expected ESP32 serial format: H,<count>,<millis>
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
            self.get_logger().info(f'Serial connected: {self.port}')
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

            # Parse structured format: H,<count>,<millis>
            if line.startswith('H,'):
                parts = line.split(',')
                if len(parts) >= 2:
                    try:
                        count = int(parts[1])
                    except ValueError:
                        return

                    # Publish on count increment (new magnet pass)
                    if count != self.last_count:
                        self.last_count = count
                        msg = Header()
                        msg.stamp = self.get_clock().now().to_msg()
                        msg.frame_id = 'hall_sensor'
                        self.trigger_pub.publish(msg)
                        self.get_logger().debug(f'Hall trigger: count={count}')

            # Also handle legacy format: count=N pin=LOW/HIGH
            elif line.startswith('count='):
                parts = line.split()
                for part in parts:
                    if part.startswith('count='):
                        try:
                            count = int(part.split('=')[1])
                        except (ValueError, IndexError):
                            continue

                        if count != self.last_count:
                            self.last_count = count
                            msg = Header()
                            msg.stamp = self.get_clock().now().to_msg()
                            msg.frame_id = 'hall_sensor'
                            self.trigger_pub.publish(msg)
                            self.get_logger().debug(f'Hall trigger (legacy): count={count}')

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
