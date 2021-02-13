#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('/ego_id/scan', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/ego_id/drive', AckermannDriveStamped, queue_size=100)

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = list(ranges)
        max_accepted_distance = 3
        
        for i in range(len(ranges)):
            if (math.isnan(ranges[i])):
                proc_ranges[i] = 0
            elif ((ranges[i] > max_accepted_distance) or math.isinf(ranges[i])):
                proc_ranges[i] = max_accepted_distance
            else:
                proc_ranges[i] = ranges[i]
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        
        max_begin = 0
        max_end = 0
        current_gap = 0
        current_begin = 0
        current_index = 0
        max_gap = 0

        while(current_index < len(free_space_ranges)):
            
            current_gap = 0
            current_begin = current_index

            while ((current_index < len(free_space_ranges)) 
                and (free_space_ranges[current_index] > 0.1)):
                current_gap+=1
                current_index+=1
            
            if (current_gap > max_gap):
                max_gap = current_gap
                max_begin = current_begin
                current_gap = 0

            current_index+=1

        if (current_gap > max_gap):
            max_gap = current_gap
            max_begin = current_begin

        return max_begin, max_begin + max_gap - 1 
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        best_point = start_i

        for i in range(start_i, end_i):
            if (ranges[i] > best_point):
                best_point = ranges[i]

        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm 
        & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        closest_point = proc_ranges[0]
        for i in range(len(proc_ranges)):
            if (proc_ranges[i] < closest_point):
                closest_point = proc_ranges[i]

        #Eliminate all points inside 'bubble' (set them to zero) 
        ratio_of_bubble_to_ranges = 10
        bubble_radius = (len(proc_ranges) / ratio_of_bubble_to_ranges) / 2
        for i in range(int(max(0, closest_point - bubble_radius)), 
	        int(min(closest_point + bubble_radius, len(proc_ranges) - 1))):
            proc_ranges[i] = 0

        #Find max length gap 
        start_point, end_point = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_point = self.find_best_point(start_point, end_point, proc_ranges)

        #Steering angle
        best_steering_angle = 0
        if (best_point < len(proc_ranges) / 2):
            distance = (len(proc_ranges) / 2) - best_point
            best_steering_angle = - distance * data.angle_increment
        else:
            distance = (best_point - (len(proc_ranges) / 2))
            best_steering_angle = distance * data.angle_increment

        #Publish Drive message
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "laser"
        ack_msg.drive.steering_angle = best_steering_angle

        multiplier = 1
        delta = abs(2 - ack_msg.drive.speed)

        ack_msg.drive.speed = 1 / (1 + np.exp(-multiplier * delta))

        print("Speed: ", ack_msg.drive.speed)
        print("Steering angle: ", best_steering_angle)

        # ack_msg.drive.acceleration = 2
        self.drive_pub.publish(ack_msg)

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
