import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        proc_lidarscan_topic = '/scan_proc'

        self._laser_scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self._cmd_drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self._proc_laser_scan_pub = self.create_publisher(LaserScan, proc_lidarscan_topic, 10)
        
        self.forward_range = np.deg2rad(60)
        self.disparity_threshold = 0.5
        self.agent_width = (0.2032 + 0.0381*2)
        self.rb = self.agent_width * 0.4
        self.get_logger().info("HI FROM INIT")

    def preprocess_lidar(self, data: LaserScan):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # TODO: item #1 above
        
        proc_ranges = [r if r <= 3 else float('inf') for r in data.ranges]
        
        num_points = len(data.ranges)
        mid_point = num_points // 2
        angle_per_step = data.angle_increment
        start_index = int(mid_point - self.forward_range / angle_per_step)
        end_index = int(mid_point + self.forward_range / angle_per_step)
        
        # Set elements from the start to the start_index to inf
        for i in range(start_index):
            proc_ranges[i] = float('inf')

        # Set elements from end_index + 1 to the end of the array to inf
        for i in range(end_index + 1, len(proc_ranges)):
            proc_ranges[i] = float('inf')
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_len = 0
        max_start = 0
        max_end = 0
        current_len = 0
        current_start = None

        for i, value in enumerate(free_space_ranges):
            if value != 0 and value != float('inf'):
                if current_start is None:
                    current_start = i
                current_len += 1
            else:
                if current_len > max_len:
                    max_len = current_len
                    max_start = current_start
                    max_end = i - 1
                current_len = 0
                current_start = None

        # Check last sequence in case the longest non-zero sequence ends at the last element
        if current_len > max_len:
            max_len = current_len
            max_start = current_start
            max_end = len(free_space_ranges) - 1

        return (max_start, max_end)

    
    def find_best_point(self, start_i: int, end_i: int, ranges: list):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
                    
        best_point = start_i + ranges[start_i:end_i+1].index(max(ranges[start_i:end_i+1]))
        
        return best_point

    def lidar_callback(self, data: LaserScan):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        self.get_logger().info("HI FROM CALLBACK")
        proc_ranges = self.preprocess_lidar(data)
        
        #Find closest point to LiDAR
        closest_point_i = proc_ranges.index(min(proc_ranges))
        
        #Eliminate all points inside 'bubble' (set them to zero) 
        if closest_point_i < len(proc_ranges):

            a = proc_ranges[closest_point_i]
            for i in range(len(proc_ranges)):
                b = proc_ranges[i]
                theta = abs(closest_point_i - i)*data.angle_increment
                c = np.sqrt(a**2 + b**2 - 2*a*b*np.cos(theta))
                if c <= self.rb:
                    proc_ranges[i] = 0.

        #Find max length gap 
        max_start, max_end = self.find_max_gap(proc_ranges)
        # for i in range(len(proc_ranges)):
        #     if i < max_start or i > max_end:
        #         proc_ranges[i] = 0.
        
        # #Disparity Extender
        # for i in range(max(max_start-10,0), min(max_end+10, len(proc_ranges))):
        for i in range(1, len(proc_ranges)):
            curr_abs_range = data.ranges[i]
            last_abs_range = data.ranges[i-1]
            
            if curr_abs_range == float('inf') or curr_abs_range == 0 \
                or last_abs_range == float('inf') or last_abs_range == 0:
                    continue
            
            disparity = abs(curr_abs_range - last_abs_range)
            
            # found disparity
            if disparity > self.disparity_threshold:
                if data.ranges[i] != 0:
                    # theta = np.arccos(1-self.agent_width**2/data.ranges[i]**2*0.5)
                    # num = int(np.ceil(theta/data.angle_increment))
                    # a = max(curr_abs_range, last_abs_range)
                    # if min(curr_abs_range, last_abs_range) == curr_abs_range:
                    #     a = last_abs_range
                    #     # for j in range(num+1):
                    #     #     proc_ranges[i-j] = data.ranges[i+1]
                    # else:
                    #     a = curr_abs_range
                    #     # for j in range(num+1):
                    #     #     proc_ranges[i+j] = proc_ranges[i-1]
                    a_high = max(curr_abs_range, last_abs_range)
                    a_low = min(curr_abs_range, last_abs_range)
                    for j in range(len(proc_ranges)):
                            b = proc_ranges[j]
                            theta = abs(i-j)*data.angle_increment
                            c_high = np.sqrt(a_high**2 + b**2 - 2*a_high*b*np.cos(theta))
                            c_low = np.sqrt(a_low**2 + b**2 - 2*a_low*b*np.cos(theta))
                            if c_high <= self.agent_width*2 or c_low <= self.agent_width*2:
                                proc_ranges[j] = 0.
                else:
                    print(f"data.ranges[{i}] == 0")
                
        
        
        # for i in range(max_start, max_end+1):
        #     if i <= len(data.ranges) - 2 and (data.ranges[i+1] - data.ranges[i]) > self.disparity_threshold:
        #         if data.ranges[i] != 0:
        #             theta = np.arccos(1-self.agent_width**2/data.ranges[i]**2*0.5)
        #             num = int(np.ceil(theta/data.angle_increment))
        #             print(f"left: {num}")
        #         else:
        #             num = 3
                    
        #         for j in range(num):
        #                 if i+j >= len(data.ranges):
        #                     break
        #                 proc_ranges[i+j] = data.ranges[i]
        #     if i <= len(data.ranges) - 2 and (data.ranges[i] - data.ranges[i+1]) > self.disparity_threshold:
        #         if data.ranges[i] != 0:
        #             theta = np.arccos(1-self.agent_width**2/data.ranges[i]**2*0.5)
        #             num = int(np.ceil(theta/data.angle_increment))
        #             print(f"right: {num}")
        #         else:
        #             num = 3
                    
        #         for j in range(num):
        #             if i-j < 0:
        #                 break
        #             proc_ranges[i-j] = data.ranges[i]
                    
        #Find the best point in the gap 
        best_point_i = self.find_best_point(max_start, max_end, proc_ranges)

        #Publish Drive message
        num_points = len(data.ranges)
        mid_point = num_points // 2
        angle_per_step = data.angle_increment
        angle = angle_per_step * (best_point_i - mid_point)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = 0.5
        self._cmd_drive_pub.publish(drive_msg)
        
        #Publish processed lidar scan message
        proc_scan_msg = LaserScan()
        proc_scan_msg = data
        proc_scan_msg.ranges = proc_ranges
        self._proc_laser_scan_pub.publish(proc_scan_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()