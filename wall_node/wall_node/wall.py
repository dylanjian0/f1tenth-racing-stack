#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# PID CONTROL PARAMS
kp = 1
kd = 0.001
ki = 0.005
servo_offset = 0.0
prev_error = 0.0
integral = 0.0
prev_time = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 1.1  # meters
DESIRED_DISTANCE_LEFT = 0.65
VELOCITY = 1.5  # meters per second
CAR_LENGTH = 1.0  # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow(Node):
    """ Implement Wall Following on the car """
    def __init__(self):
        super().__init__('wall_follow')
        print("a")

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        global prev_time
        prev_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def getRange(self, data, angle):
        """
        Gets the range at a specific angle from Lidar data.
        Args:
            data: single message from topic /scan
            angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        Returns:
            Distance in meters at the given angle
        """
        if angle >= -45 and angle <= 225:
            iterator = len(data) * (angle + 90) / 360
            if not np.isnan(data[int(iterator)]) and not np.isinf(data[int(iterator)]):
                return data[int(iterator)]
        return float('inf')

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        del_time = current_time - prev_time

        # Avoid division by zero in case del_time is too small
        if del_time == 0:
            del_time = 1e-6  # A very small value to avoid division by zero

        # PID control formula
        integral += prev_error * del_time
        angle = kp * error + ki * integral + kd * (error - prev_error) / del_time

        prev_error = error
        prev_time = current_time

        # Prepare drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle

        # Adjust speed based on steering angle
        if abs(angle) <= math.radians(10):
            drive_msg.drive.speed = velocity
        elif abs(angle) <= math.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5

        # Publish drive message
        self.drive_pub.publish(drive_msg)


    def followLeft(self, data, leftDist):
        """
        Follow the left wall using two Lidar distances (front and back).
        Args:
            data: Lidar data ranges
            leftDist: Desired distance to the left wall
        Returns:
            Error: the difference between desired and current distance to the wall
        """
        front_scan_angle = 125
        back_scan_angle = 180
        teta = math.radians(abs(front_scan_angle - back_scan_angle))

        front_scan_dist = self.getRange(data, front_scan_angle)
        back_scan_dist = self.getRange(data, back_scan_angle)

        if math.isinf(front_scan_dist) or math.isinf(back_scan_dist):
            return 0.0  # Prevent any calculations if distances are invalid

        alpha = math.atan2(front_scan_dist * math.cos(teta) - back_scan_dist, 
                           front_scan_dist * math.sin(teta))
        wall_dist = back_scan_dist * math.cos(alpha)
        ahead_wall_dist = wall_dist + CAR_LENGTH * math.sin(alpha)

        return leftDist - ahead_wall_dist
        

    def lidar_callback(self, data):
        """
        Lidar callback to handle incoming scan data and control the car.
        Args:
            data: Lidar scan message
        """
        error = self.followLeft(data.ranges, DESIRED_DISTANCE_LEFT)  # Get error based on the left wall distance
        self.pid_control(error, VELOCITY)  # Pass error to PID controller


def main(args=None):
    rclpy.init(args=args)
    wf = WallFollow()
    rclpy.spin(wf)  
    wf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()