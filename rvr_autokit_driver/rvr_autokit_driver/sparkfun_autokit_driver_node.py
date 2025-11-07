#!/usr/bin/env python3

"""
Sparkfun RVR Autonomous Kit ROS 2 driver

Controls the Sparkfun RVR autonomous kit including:
- GPS
- Distance sensor (VL53L1X)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, TimeReference
import qwiic_titan_gps
import qwiic_vl53l1x


class RVRAutoKitDriverNode(Node):
    def __init__(self):
        super().__init__('rvr_autokit_driver')

        # Parameters
        self.loop_rate = float(self.declare_parameter('loop_rate', 4.0).value)

        # Initialize hardware
        self.qwiic_gps = qwiic_titan_gps.QwiicTitanGps()
        self.qwiic_vl53l1x = qwiic_vl53l1x.QwiicVL53L1X()

        # ROS message variables
        self.laser_scan = LaserScan()
        self.nav_sat_fix = NavSatFix()
        self.time_reference = TimeReference()

        # Check resources operational
        if self.qwiic_gps.connected is False:
            self.get_logger().error('Qwiic Titan GPS not connected')
        else:
            self.get_logger().info('Qwiic Titan GPS connected')

        if not self.qwiic_vl53l1x.sensor_init():
            self.get_logger().error('Qwiic VL53L1X not connected')
        else:
            self.get_logger().info('Qwiic VL53L1X connected')

        # Publishers
        self.laser_scan_pub = self.create_publisher(LaserScan, 'scan', 1)
        self.nav_sat_fix_pub = self.create_publisher(NavSatFix, 'gps', 1)
        self.time_reference_pub = self.create_publisher(
            TimeReference, 'time_ref', 1)

        # Start hardware
        self.qwiic_gps.begin()

        # Timer for sensor polling
        timer_period = 1.0 / self.loop_rate if self.loop_rate > 0 else 0.25
        self.timer = self.create_timer(timer_period, self.check_sensor_data)

        self.get_logger().info('RVR AutoKit driver initialized')

    def check_sensor_data(self):
        """Poll sensors and publish data"""

        # Get distance sensor data
        try:
            self.qwiic_vl53l1x.start_ranging()
            # Small delay for sensor reading
            import time
            time.sleep(0.005)
            distance = self.qwiic_vl53l1x.get_distance() / 1000.0  # Convert mm to meters
            time.sleep(0.005)
            self.qwiic_vl53l1x.stop_ranging()

            # Populate laser scan message
            self.laser_scan.header.frame_id = 'laser'
            self.laser_scan.header.stamp = self.get_clock().now().to_msg()
            self.laser_scan.angle_min = 0.0
            self.laser_scan.angle_max = 0.0
            self.laser_scan.angle_increment = 0.0
            self.laser_scan.time_increment = 0.0
            self.laser_scan.scan_time = 0.0
            self.laser_scan.range_min = 0.0
            self.laser_scan.range_max = 4.0  # VL53L1X max range ~4m
            self.laser_scan.ranges = [float(distance)]
            self.laser_scan.intensities = [1.0]
        except Exception as e:
            self.get_logger().warn(f'Distance sensor error: {e}')

        # Get GPS data
        try:
            if self.qwiic_gps.get_nmea_data() is True:
                gps_time = self.qwiic_gps['Time']

                self.nav_sat_fix.header.frame_id = 'gps'
                # Convert GPS time to ROS time
                timestamp = self.get_clock().now()
                self.nav_sat_fix.header.stamp = timestamp.to_msg()
                self.nav_sat_fix.latitude = float(self.qwiic_gps['Latitude'])
                self.nav_sat_fix.longitude = float(self.qwiic_gps['Longitude'])
                self.nav_sat_fix.altitude = float(self.qwiic_gps['Altitude'])
                self.nav_sat_fix.position_covariance = [0.0] * 9
                self.nav_sat_fix.position_covariance_type = 0

                # Time reference
                self.time_reference.header.frame_id = 'gps'
                self.time_reference.header.stamp = timestamp.to_msg()
                self.time_reference.time_ref = self.nav_sat_fix.header.stamp
        except Exception as e:
            self.get_logger().warn(
                f'GPS error: {e}', throttle_duration_sec=5.0)

        # Publish data
        self.laser_scan_pub.publish(self.laser_scan)
        self.nav_sat_fix_pub.publish(self.nav_sat_fix)
        self.time_reference_pub.publish(self.time_reference)


def main(args=None):
    rclpy.init(args=args)
    node = RVRAutoKitDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
