#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

# include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):

    def __init__(self):
        super().__init__("safety_node")

        self.speed = 0.0
        self.emergency_braking = False
        self.scan_subscription = (
            self.create_subscription(
                LaserScan, "/scan", self.scan_callback, 10
            )
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/ego_racecar/odom",
            self.odom_callback,
            10,
        )

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 
        "/drive", 1000)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        
        emergency_detected = False
        for idx, r in enumerate(scan_msg.ranges):
            if np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min:
                continue

            threshold = 1  # seconds
            iTTC = r / max(
                abs(self.speed * np.cos(scan_msg.angle_min + idx * scan_msg.angle_increment)),
                0.001
            )

            if iTTC < threshold:
                emergency_detected = True
                break

        if emergency_detected and not self.emergency_braking:
            self.emergency_braking = True
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.get_logger().info(f"Emergency brake engaged at speed {self.speed}")
            self.publisher_.publish(drive_msg)

        elif not emergency_detected and self.emergency_braking:
            # Reset the emergency brake state when the obstacle is cleared
            self.emergency_braking = False
            self.get_logger().info("Emergency situation cleared.")

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()