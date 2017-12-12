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


#def cluster()
def callback(scan):
    # if  scan.intensities[0] > 0:
    # intensities 360 length 0 or 47
    # scan_time 7.96710082795e-05
    # ranges tuple float32
    # angle_min -3.12413907051
    # 3.14159274101
    # angle_increment 0.0157
    global ranges_filtered
    global count
    global num_accumulated
    ranges_np = np.array(scan.ranges)
    ranges_np[ranges_np == inf] = 0
    ranges_np[ranges_np > 4] = 0
    ranges_angle = 80
    ranges_np[ranges_angle + 1:360 - ranges_angle] = 0
    num_accumulated = 5
    if count < num_accumulated:
    	ranges_filtered[:, count] = ranges_np
    	count += 1
    else:
        ranges_median = np.median(ranges_filtered, axis = 1)
	print(ranges_median.shape)
        ranges_scan = np.zeros(2*ranges_angle)
        ranges_scan[0:ranges_angle] = ranges_median[360 - ranges_angle : 360]
        ranges_scan[ranges_angle : 2 * ranges_angle] = ranges_median[0:ranges_angle]
        i = 0
        index = 0
        block_count = 0
        block_index = np.array([])
        while i < 2 * ranges_angle:
            if ranges_scan[i] > 0:
                index = int(np.ceil(0.2 / ranges_scan[i]  / 0.0175))
                flag = 0
                for j in range(index):
                    if i+j > 2 * ranges_angle-1:
                        break
                    if abs(ranges_scan[i] - ranges_scan[i+j]) < 0.2 and ranges_scan[i+j] > 0: 
                        flag = j
                if flag > 0:
                    
                    # to do: change the idex to flag
                    block_count += 1
                    block_index= np.append(block_index, int(i +flag/2))
                    ranges_scan[i:i+index] = ranges_scan[i]
                    i = i + index
                else:
                    i += 1
            else:
                i += 1

        block_index = np.mod((360 + block_index - ranges_angle), 360).astype('int')
        block_position = np.zeros((30 * 3))
        block_position[0:block_count] = ranges_median[block_index] * np.cos(block_index.astype('float') / 180 * np.pi)
        block_position[block_count:2 * block_count] = ranges_median[block_index] * np.sin(block_index.astype('float') / 180 * np.pi)
        block_position[2*block_count: 3*block_count] = range(block_count)

        location = Float32MultiArray()
        location.data = block_position
        # print(block_position)
        publisher_location(location)
        count = 0
        ranges_filtered = np.zeros((360,num_accumulated))
        '''
	ranges_median[360 - ranges_angle:360] = ranges_scan[0 : ranges_angle]
        ranges_median[0:ranges_angle] = ranges_scan[ranges_angle  : 2 * ranges_angle]
        
        point_x, point_y = rangeToPoint(ranges_median)
        point_matrix = np.append(point_x,point_y, axis=0)
	print(point_matrix.shape)
        eps = 0.2
        min_points = 2
       

        label = dbs.dbscan(point_matrix, eps, min_points)
        delete the 0-0 point
	label_max = np.max(label)
        label_count = 1
	location = np.zeros((label_max-1, 3))
        
        for i in range(360):
            if ((label[i] - 1) == label_count):
                print(point_x.shape)

		location[label_count,:] = np.array([point_x[i], point_y[i], label_count])
                label_count += 1 
        publisher_location(location)
        
        #plt.plot(point_x, point_y, 'ro')
        #plt.axis([-5, 5, -5, 5])
	#plt.show()
	 
        scan_median = scan
	scan_median.ranges = tuple(ranges_median)
        # rospy.loginfo(scan_augmented.ranges)
	publisher(scan_median)	
	
        
        '''
	
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
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
def publisher_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_location',Float32MultiArray , queue_size=1000)
    pub.publish(location)
if __name__ == '__main__':
    # global num_accumulated
    
    listener()
    
