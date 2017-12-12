#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import inf
import dbscan as dbs
from std_msgs.msg import Float32MultiArray
import time

global num_accumulated
global ranges_filtered_accumulate
global ranges_filtered_window
global count 

num_accumulated = 5
ranges_filtered_accunulate = np.zeros((360,num_accumulated))
ranges_filtered_window = np.zeros((360,num_accumulated))
count = 0


def rangeToPoint(ranges,front=270):
	
        angle = (front - np.arange(360) ) * np.pi * 2 / 360
	point_x = ranges * np.sin(angle)
        # print(np.array(point_x).shape)
	point_y = ranges * np.cos(angle)
	return point_x, point_y

def ranges_index_to_point(center_ranges, center_index):
    angle = center_index * np.pi * 2 / 360
    center = np.zeros(2)
    center[0] = center_ranges * np.sin(angle)
    center[1] = center_ranges * np.cos(angle)
    return center

def ranges_left_right_to_point(center_ranges, left_index, right_index):
    angle = (270-(left_index + right_index) / 2 )* np.pi * 2 / 360
    center = np.zeros(2)
    center[0] = center_ranges * np.sin(angle)
    center[1] = center_ranges * np.cos(angle)
    return center



def box_filter(ranges_np, front=270, left_x=-0.5, left_y=0, right_x=0.5, right_y=2):
    point_x, point_y = rangeToPoint(ranges_np, front)
    for i in range(360):
        
        if point_x[i] > left_x and point_x[i] < right_x and point_y[i] > left_y and point_y[i] < right_y:
            pass
        else:
            ranges_np[i] = 0 
    return ranges_np

def median_accumulate(ranges_np,accumulate_num = 5):
    global count
    global ranges_filtered_accumulate
    global num_accumulated
    
    if count < num_accumulated:
        ranges_filtered_accumulate[:, count] = ranges_np
        count += 1
    else:
        ranges_median = np.median(ranges_filtered_accumulate, axis = 1)
        ranges_filtered_accumulate = np.zeros((360,num_accumulated))
        count = 0
    return ranges_median
    
def median_window(ranges_np):
    global ranges_filtered_window
    global num_accumulated    
    temp = ranges_filtered_window[:,1:num_accumulated]
    ranges_filtered_window[: , 0 : num_accumulated - 1] = temp
    ranges_filtered_window[: , num_accumulated-1] = ranges_np
    ranges_median = np.median(ranges_filtered_window, axis = 1)
    return ranges_median

def center_to_entire(index,ranges_angle = 70, front=270):
    temp = index - ranges_angle + front
    return temp

def callback(scan):
    
    global ranges_filtered
    global num_accumulated
    
    #case select
    flag_ranges = 1    # 1 for sliding window, 0 for accumulate
    flag_scan_publish = 0
    distance = 1.5
    ranges_angle = 70
    front = 270 
    left_x = -0.5
    left_y = 0
    right_x = 0.5
    right_y = 1.2
    flag_choose = 0 # 1 for find the centeric one, 0 for finding nearest one.
    # filter some regions
    ranges_np = np.array(scan.ranges)
    ranges_np[ranges_np == inf] = 0    
    ranges_np[ranges_np > distance] = 0   
     
    ranges_np[0 : front- ranges_angle] = 0
    ranges_np[front + ranges_angle : 360] = 0    
    ranges_np = box_filter(ranges_np, front, left_x, left_y, right_x, right_y)
    num_accumulated = 5
    center = []
    center_index = 0
    
    if flag_ranges == 1:
        ranges_median = median_window(ranges_np)
    else:
        ranges_median = median_accumulate(ranges_np)
        
    ranges_scan = np.zeros(2*ranges_angle)
    ranges_scan = ranges_median[front - ranges_angle+1: front + ranges_angle+1]
    i = 0
    index = 0
    
    flag_block = 0
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
                if flag_block == 0:
                    ranges_scan[i:i+index] = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2)
                    center_ranges = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2)                
                    left_index = center_to_entire(i,ranges_angle,front)
                    right_index = center_to_entire(i+flag, ranges_angle,front) 
                    center_index = np.floor((left_index + right_index)/2)
                    center = ranges_left_right_to_point(center_ranges, left_index, right_index)
                    print ('i is {}'.format(i))
                    print('flag is {}'.format(flag))
                    i = i + index
                    flag_block = 1 
                else: 
                    temp_left_index =  center_to_entire(i,ranges_angle,front)
                    temp_right_index = center_to_entire(i+flag, ranges_angle,front)
                    temp_center_index = np.floor((temp_left_index + temp_right_index)/2) 
                    temp_center_ranges = np.sqrt(ranges_scan[i] ** 2 - 0.1 ** 2) 
                    if flag_choose == 1:
                        # choose the most centric one.
                        if abs(temp_center_index - front) <  abs(center_index - front):
                            center_index =  temp_center_index
                            center_ranges = temp_center_ranges
                            left_index = temp_left_index
                            right_index = temp_right_index
                    else:
                        # choose the nearest one.
                        if  temp_center_ranges < center_ranges:
                            center_index =  temp_center_index
                            center_ranges = temp_center_ranges
                            left_index = temp_left_index
                            right_index = temp_right_index
                        
                    
                    center = ranges_left_right_to_point(center_ranges, left_index, right_index)
                    i = i + index
            else:
                i += 1
        else:
            i += 1
    range_median = np.zeros(360)
    ranges_median[front - ranges_angle+1: front + ranges_angle+1] = ranges_scan
    location = Float32MultiArray()
    location.data = center
    print(center)
    print(center_index)
    publisher_single_location(location)
    
    if flag_scan_publish == 1:
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
