#!/usr/bin/env python3

"""
Sparkfun RVR Autonomous Kit ROS 2 driver

Controls the Sparkfun RVR autonomous kit including:
- GPS
- Distance sensor (VL53L1X)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import TimeReference
from sensor_msgs.msg import Range
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
        self.range_msg = Range()
        self.range_msg.radiation_type = Range.INFRARED
        self.range_msg.field_of_view = 0.0
        self.range_msg.min_range = 0.03
        self.range_msg.max_range = 4.0
        self.nav_sat_fix = NavSatFix()
        self.nav_sat_fix.status = NavSatStatus()  # Initialize status field
        self.time_reference = TimeReference()

        # Check resources operational
        if self.qwiic_gps.connected is False:
            self.get_logger().error('Qwiic Titan GPS not connected')
        else:
            self.get_logger().info('Qwiic Titan GPS connected')

        self.vl53l1x_ranging = False
        if not self.qwiic_vl53l1x.sensor_init():
            self.get_logger().error('Qwiic VL53L1X not connected')
        else:
            self.get_logger().info('Qwiic VL53L1X connected')
            try:
                self.qwiic_vl53l1x.start_ranging()
                self.vl53l1x_ranging = True
            except Exception as e:
                self.get_logger().warn(f'VL53L1X start ranging failed: {e}')

        # Publishers
        self.range_pub = self.create_publisher(
            Range, 'collision/rear/distance', 1)
        self.nav_sat_fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 1)
        self.time_reference_pub = self.create_publisher(
            TimeReference, 'gps/time_ref', 1)

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
            if not self.vl53l1x_ranging:
                self.qwiic_vl53l1x.start_ranging()
                self.vl53l1x_ranging = True

            distance = self.qwiic_vl53l1x.get_distance() / 1000.0
            self.range_msg.header.frame_id = 'autokit'
            self.range_msg.header.stamp = self.get_clock().now().to_msg()
            self.range_msg.range = float(distance)
        except Exception as e:
            self.get_logger().warn(f'Distance sensor error: {e}')

        # Get GPS data
        try:
            if self.qwiic_gps.get_nmea_data() is True:
                # GPS data is in the gnss_messages dictionary
                self.nav_sat_fix.header.frame_id = 'autokit'
                # Use ROS time for now (GPS time would require conversion from UTC list)
                timestamp = self.get_clock().now()
                self.nav_sat_fix.header.stamp = timestamp.to_msg()

                # Extract GPS position data
                self.nav_sat_fix.latitude = float(
                    self.qwiic_gps.gnss_messages['Latitude'])
                self.nav_sat_fix.longitude = float(
                    self.qwiic_gps.gnss_messages['Longitude'])

                # Altitude may not always be available
                if 'Altitude' in self.qwiic_gps.gnss_messages:
                    self.nav_sat_fix.altitude = float(
                        self.qwiic_gps.gnss_messages['Altitude'])
                else:
                    self.nav_sat_fix.altitude = 0.0

                # Set covariance (unknown, so use zeros)
                self.nav_sat_fix.position_covariance = [0.0] * 9
                self.nav_sat_fix.position_covariance_type = 0  # COVARIANCE_TYPE_UNKNOWN

                # Set status
                # STATUS_FIX (assuming fix if data is valid)
                self.nav_sat_fix.status.status = 0
                self.nav_sat_fix.status.service = 1  # SERVICE_GPS

                # Time reference (GPS time is in UTC as [hh, mm, ss])
                self.time_reference.header.frame_id = 'autokit'
                self.time_reference.header.stamp = timestamp.to_msg()
                self.time_reference.time_ref = self.nav_sat_fix.header.stamp
                self.time_reference.source = 'autokit'
        except Exception as e:
            self.get_logger().warn(
                f'GPS error: {e}', throttle_duration_sec=5.0)

        # Publish data
        self.range_pub.publish(self.range_msg)
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
