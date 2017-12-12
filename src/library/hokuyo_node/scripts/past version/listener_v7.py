#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import inf
# from matplotlib import pyplot as plt
import dbscan as dbs
from std_msgs.msg import Float32MultiArray

global ranges_filtered 
global num_accumulated
num_accumulated = 5
ranges_filtered = np.zeros((360,num_accumulated))
global count 
count = 0
def rangeToPoint(ranges):
	# if not work, try use np.array transform
        angle = np.arange(360) * np.pi * 2 / 360
	point_x = [ranges * np.sin(angle)]
	point_y = [ranges * np.cos(angle)]
	return point_x, point_y

def ranges_index_to_point(center_ranges, center_index):
    angle = center_index * np.pi * 2 / 360
    center = np.zeros(2)
    center[0] = center_ranges * np.sin(angle)
    center[1] = center_ranges * np.cos(angle)
    return center

def ranges_lft_right_to_point(center_ranges, left_index, right_index):
    angle = (left_index + right_index) / 2 * np.pi * 2 / 360
    center = np.zeros(2)
    center[0] = center_ranges * np.sin(angle)
    center[1] = center_ranges * np.cos(angle)
    return center

def callback(scan):

    global ranges_filtered
    global count
    global num_accumulated
    ranges_np = np.array(scan.ranges)
    ranges_np[ranges_np == inf] = 0
    ranges_np[ranges_np > 1] = 0
    ranges_angle = 70
    offset_angle = 0
    ranges_np[0 : 90+offset_angle- ranges_angle] = 0
    ranges_np[90+offset_angle+ ranges_angle : 360] = 0
    num_accumulated = 5
    if count < num_accumulated:
    	ranges_filtered[:, count] = ranges_np
    	count += 1
    else:
        ranges_median = np.median(ranges_filtered, axis = 1)
        ranges_scan = np.zeros(2*ranges_angle)
        ranges_scan = ranges_median[90 - ranges_angle+1 + offset_angle: 90 + ranges_angle+1 + offset_angle]
        ranges_scan = ranges_scan[::-1]
        i = 0
        index = 0
        center = []
        center_index = 0
        while i < 2 * ranges_angle:
            if ranges_scan[i] > 0:
                index = int(np.ceil(0.2 / ranges_scan[i]  / 0.0175)) + 2
                flag = 0
                for j in range(index):
                    if i+j > 2 * ranges_angle-1:
                        break
                    if abs(ranges_scan[i] - ranges_scan[i+j]) < 0.25 and ranges_scan[i+j] > 0: 
                        flag = j
                if flag > 0 and flag > index / 3:
                    
                    ranges_scan[i:i+index] = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2)
                    center_ranges = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2)
                    center_index = center_index - 90 +offset_angle
                    left_index = i - 90 +offset_angle
                    right_index = i+flag - 90 + offset_angle
                    center = ranges_lft_right_to_point(center_ranges, left_index, right_index)
                    print ('i is {}'.format(i))
                    print('flag is {}'.format(flag))
                    i = i + index
                else:
                    i += 1
            else:
                i += 1
        ranges_scan = ranges_scan[::-1]
        range_median = np.zeros(360)
        ranges_median[90 - ranges_angle+1 + offset_angle: 90 + ranges_angle+1 + offset_angle] = ranges_scan

        location = Float32MultiArray()
        location.data = center
        print('center_index is {}'.format(center_index))
        print('center is {}'.format(center))
        publisher_single_location(location)
        count = 0
        scan_median = scan
	scan_median.ranges = tuple(ranges_median)
        # rospy.loginfo(scan_augmented.ranges)
	publisher(scan_median)	
        ranges_filtered = np.zeros((360,num_accumulated))
        
	
def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
	
def publisher(scan_median):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/scan_median', LaserScan, queue_size=1000)
    
    pub.publish(scan_median)
    
def publisher_multiple_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_multiple_location',Float32MultiArray , queue_size=1000)
    pub.publish(location)

def publisher_single_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_single_location',Float32MultiArray , queue_size=1000)
    pub.publish(location)

if __name__ == '__main__':
    # global num_accumulated
    
    listener()
    
