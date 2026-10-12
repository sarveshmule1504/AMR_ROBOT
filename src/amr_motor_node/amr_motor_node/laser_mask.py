#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserMaskNode(Node):
    def __init__(self):
        super().__init__('laser_mask_node')

        # Declare parameters for 4 masking sectors (Start and End in DEGREES)
        # Default ranges are set to 0.0, 0.0 (disabled by default)
        self.declare_parameter('mask_zone1', [42.5, 47.5])
        self.declare_parameter('mask_zone2', [132.5, 137.5])
        self.declare_parameter('mask_zone3', [222.5, 227.5])
        self.declare_parameter('mask_zone4', [312.5, 317.5])
        self.declare_parameter('input_topic', '/raw_scan')
        self.declare_parameter('output_topic', '/scan')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # QoS profile matching standard SensorData QOS
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            LaserScan,
            output_topic,
            10
        )

        self.get_logger().info(f"Laser Mask Node started subscribing to {input_topic}...")

    def normalize_angle_deg(self, angle_deg):
        """ Normalizes an angle in degrees to range [-180, 180]. """
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def is_angle_masked(self, angle_deg, start_deg, end_deg):
        """ Checks if a given angle falls inside [start_deg, end_deg]. """
        # Ignore disabled/empty zones
        if start_deg == 0.0 and end_deg == 0.0:
            return False

        angle = self.normalize_angle_deg(angle_deg)
        start = self.normalize_angle_deg(start_deg)
        end = self.normalize_angle_deg(end_deg)

        # Handle range wrap-around across the -180/180 boundary (e.g. start=170, end=-170)
        if start <= end:
            return start <= angle <= end
        else:
            return angle >= start or angle <= end

    def scan_callback(self, msg: LaserScan):
        # Fetch current parameter values
        zones = [
            self.get_parameter('mask_zone1').get_parameter_value().double_array_value,
            self.get_parameter('mask_zone2').get_parameter_value().double_array_value,
            self.get_parameter('mask_zone3').get_parameter_value().double_array_value,
            self.get_parameter('mask_zone4').get_parameter_value().double_array_value,
        ]

        # Convert ranges and intensities to lists for modification
        masked_ranges = list(msg.ranges)
        has_intensities = len(msg.intensities) > 0
        masked_intensities = list(msg.intensities) if has_intensities else []

        angle_min_deg = math.degrees(msg.angle_min)
        angle_increment_deg = math.degrees(msg.angle_increment)

        # Iterate over all lidar beams
        for i in range(len(masked_ranges)):
            current_angle_deg = angle_min_deg + (i * angle_increment_deg)

            # Check if current angle lies within any of the 4 defined zones
            for zone in zones:
                if len(zone) == 2 and self.is_angle_masked(current_angle_deg, zone[0], zone[1]):
                    masked_ranges[i] = float('inf')  # Mask range by setting to inf
                    if has_intensities:
                        masked_intensities[i] = 0.0
                    break

        # Reassign masked data to output message
        masked_msg = msg
        masked_msg.ranges = masked_ranges
        if has_intensities:
            masked_msg.intensities = masked_intensities

        self.publisher_.publish(masked_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserMaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
