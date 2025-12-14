#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )
        self.get_logger().info("Subscribed to /scan!")

    def callback(self, msg):
        self.get_logger().info(f"First range: {msg.ranges[0]}")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
