#ifndef HOKUYO_PROCESS_H
#define HOKUYO_PROCESS_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <iostream>


class hokuyo
{
private:
    ros::NodeHandlePtr node_;
    void InitParameters(ros::NodeHandlePtr node_para);
public:
    hokuyo(ros::NodeHandle &nh, ros::NodeHandlePtr node_ptr);
    ros::Subscriber lidar_proc_sub;
    ros::Publisher block_pub;

    void ScanCallback(const sensor_msgs::LaserScan& scan);
    int blind_detector(std::vector<cv::Point2f>& raw_data, cv::Point2f& median_points);
    int lsd_detector(std::vector<cv::Point2f>& raw_data,
                     std::vector<cv::Point2f>& median_points,
                     float angle);
    std::vector<std::array<float, 4> > lineEndpoints_;


private:
    bool debug; // show debug info
    bool show;  // show real-time video
    bool debug_raw; //show raw data

    int lidar_img_height; // y-axis 160
    int lidar_img_width;  // x-axis 250

    int box_left_right;   // 70 in left and right respectively
    int box_top_bottom;   // 240 in vertical distance

    /// NOTE:once kernal size changed, the thresh should be changed
    int sobel_kernal_size; // 3 is preferred
    int sobel_thres_value; // 600-1000 is okay

    int min_line_point_num; // at least 10 points are founded
    double min_x_dis; // tolerance of x direction of lidar
    int ignore_num; // tolerance of y direction lo lidar

};


#endif
