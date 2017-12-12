#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import inf
# from matplotlib import pyplot as plt
import dbscan as dbs
from std_msgs.msg import Float32MultiArray
import time

global ranges_filtered 
global num_accumulated
num_accumulated = 10
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
#def cluster()
def callback(scan):
    #print(scan)
    
    # if  scan.intensities[0] > 0:
    # intensities 360 length 0 or 47
    # scan_time 7.96710082795e-05
    # ranges tuple float32
    # angle_min -3.12413907051
    # 3.14159274101
    # angle_increment 0.0157
    global ranges_filtered
    global num_accumulated
    ranges_np = np.array(scan.ranges)
    ranges_np[ranges_np == inf] = 0
    ranges_np[ranges_np > 1] = 0
    ranges_angle = 80
    ranges_np[ranges_angle + 1:360 - ranges_angle] = 0
    num_accumulated = 10
    center = []
    center_index = 0
    if 1>0:
        temp = ranges_filtered[:,1:num_accumulated]
        ranges_filtered[: , 0 : num_accumulated - 1] = temp
        ranges_filtered[: , num_accumulated-1] = ranges_np
        ranges_median = np.median(ranges_filtered, axis = 1)
        ranges_scan = np.zeros(2*ranges_angle)
        ranges_scan[0:ranges_angle] = ranges_median[360 - ranges_angle:360]
        ranges_scan[ranges_angle : 2 * ranges_angle] = ranges_median[0:ranges_angle]
        ranges_scan = ranges_scan[::-1]
        i = 0
        index = 0
        center_index = 0
        center = []
        while i < 2 * ranges_angle:
            if ranges_scan[i] > 0:
                index = int(np.ceil(0.2 / ranges_scan[i]  / 0.0175)) + 2
                flag = 0
                for j in range(index):
                    if i+j > 2 * ranges_angle-1:
                        break
                    if abs(ranges_scan[i] - ranges_scan[i+j]) < 0.25 and ranges_scan[i+j] > 0: 
                        flag = j
                if flag > 0 and flag > index/2 :
                    ranges_scan[i:i+index] = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2)
                    center_ranges = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2)
                    center_index = np.floor(i+flag/2)
                    print(i)
                    print(index)
                    center_index = np.mod((360 + center_index - ranges_angle), 360).astype('int')
                    left_index = np.mod((360 + i - ranges_angle), 360).astype('int')
                    right_index = np.mod((360 + i + flag - ranges_angle), 360).astype('int')
                    center = ranges_lft_right_to_point(center_ranges, left_index, right_index)
                    i = i + index
                               
                else:
                    ranges_scan[i] = 0
                    i += 1
            else:
                i += 1
        range_median = np.zeros(360)
        ranges_scan = ranges_scan[::-1]
        ranges_median[360 - ranges_angle:360] = ranges_scan[0 : ranges_angle]
        ranges_median[0:ranges_angle] = ranges_scan[ranges_angle  : 2 * ranges_angle]
        location = Float32MultiArray()
        location.data = center
        print(center)
        print(center_index)
        publisher_single_location(location)
        scan_median = scan
	scan_median.ranges = tuple(ranges_median)
	publisher(scan_median)	

      
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
	
def publisher(scan_median):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/scan_median', LaserScan, queue_size=1000)
    # rate = rospy.Rate(10) # 0.5hz
    pub.publish(scan_median)
    # rate.sleep()
def publisher_multiply_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_multiply_location',Float32MultiArray , queue_size=1000)
    pub.publish(location)
def publisher_single_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_single_location',Float32MultiArray , queue_size=1000)
    pub.publish(location)
if __name__ == '__main__':
    # global num_accumulated
    
    listener()
    '''
        
        for i in range(360):
            if ((label[i] - 1) == label_count):
                print(point_x.shape)

		location[label_count,:] = np.array([point_x[i], point_y[i], label_count])
                label_count += 1 
'''
