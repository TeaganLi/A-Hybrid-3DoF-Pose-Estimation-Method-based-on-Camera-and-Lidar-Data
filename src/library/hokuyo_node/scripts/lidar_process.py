#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy as np
from numpy import inf
from numpy import nan
from std_msgs.msg import Float32MultiArray
import time
import cv2
from matplotlib import pyplot as plt

debug = False
name = 0


def lidar_processing(scan):
    global debug
    global name
    length = len(scan.ranges)
    if length == 0:
        rospy.DEBUG("empty data received!")
        return
    # generate angular coordinate (2-d location)
    angular = [scan.angle_min + scan.angle_increment*i for i in xrange(length)]
    rad = np.asarray(angular)
    dis = np.asarray(scan.ranges)
    dis[abs(dis) == inf] = 100
    loc = np.array([np.cos(rad), np.sin(rad)])*dis
    if debug:
        print rad.shape, dis.shape, loc.shape

    time1 = time.time()
    # generate a index table
    liar_img = np.zeros([250, 160, 3], dtype=np.float32)
    loc_orig_x = []
    loc_orig_y = []
    for j in xrange(length - 1):
        # y axis left-right [-70 +70] [0 240]
        if abs(loc[1, j]) < 0.7 and abs(loc[0, j]) < 2.4:
            # shift y to [0 140] then reflect
            img_x = int(np.rint(loc[0, j] * 100))
            img_y = int(70 - np.rint(loc[1, j] * 100))
            # store two coordinate elements
            liar_img[img_x, img_y, 0] = loc[0, j]
            liar_img[img_x, img_y, 1] = loc[1, j]
            liar_img[img_x, img_y, 2] = 255
            # store original data
            loc_orig_x.append(loc[0, j])
            loc_orig_y.append(loc[1, j])
    time2 = time.time()

    loc_copy = (abs(loc[0, :])) * 100
    # local-extreme-signal filter
    extreme_filter = np.array([-1, 1])
    mag = abs(np.convolve(loc_copy, extreme_filter, mode='same'))
    # mag[mag < 5] = 0
    print loc_copy
    # detect_filter = np.array([-1, 1])
    # mag_diff = abs(np.convolve(mag, detect_filter))
    # print mag_diff

    print time2 - time1

    # print dis_x_array, dis_y_array
    #
    # img = liar_img[:, :, 2]
    # # calculate the magnitude degree in x and y
    # # grad_x = cv2.Sobel(liar_img[img_x, img_y, 2],)
    #
    # # show two images
    # img_show_x_value = liar_img[:, :, 0]*1000
    # img_show_x_rgb = cv2.cvtColor(img_show_x_value, cv2.COLOR_GRAY2RGB)
    # img_show_x_int = cv2.convertScaleAbs(img_show_x_rgb)
    # img_show_y_value = liar_img[:, :, 1]
    # cv2.imshow("x", img_show_x_int)
    # cv2.waitKey(1)
    # name += 1
    # file_name = "/home/zhudelong/Challenges/ICRA2017/CUApes/data/" + str(name) + ".jpg"
    # # cv2.imwrite(file_name, img)


def ros_main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_processing)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def publisher(scan_median):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/scan_median', LaserScan, queue_size=1)
    # rate = rospy.Rate(10) # 0.5hz
    pub.publish(scan_median)
    # rate.sleep()


def publisher_multiply_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_multiply_location', Float32MultiArray, queue_size=1)
    pub.publish(location)


def publisher_single_location(location):
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/block_single_location',Float32MultiArray , queue_size=1)
    pub.publish(location)


def test_image():
    img = cv2.imread('/home/zhudelong/Challenges/ICRA2017/CUApes/data/35.jpg',0)
    laplacian = cv2.Laplacian(img,cv2.CV_64F)
    sobelx = cv2.Sobel(img,cv2.CV_32F, 1, 0, ksize=3)
    sobely = cv2.Sobel(img,cv2.CV_32F, 0, 1, ksize=3)

    print np.shape(sobelx)
    for i in xrange(np.shape(sobely)[0]):
        for j in xrange(np.shape(sobely)[1]):
            if sobely[i,j] < 800:
                sobely[i,j] = 0

    img = cv2.medianBlur(sobely, ksize=3)


    print np.max(sobelx)
    print np.min(sobelx)

    plt.subplot(2,2,1),plt.imshow(img,cmap = 'gray')
    plt.title('Original'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,2),plt.imshow(laplacian,cmap = 'gray')
    plt.title('Laplacian'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,3),plt.imshow(sobelx,cmap = 'gray')
    plt.title('Sobel X'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,4),plt.imshow(sobely,cmap = 'gray')
    plt.title('Sobel Y'), plt.xticks([]), plt.yticks([])

    plt.show()


if __name__ == '__main__':
    # global num_accumulated
    ros_main()
    # test_image()

