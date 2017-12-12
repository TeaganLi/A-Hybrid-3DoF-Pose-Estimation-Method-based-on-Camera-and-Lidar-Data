#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "target_tag.h"
#include "common/getopt.h"

using namespace std;
using namespace cv;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "visual_detector");
    ros::NodeHandle nh;
    ros::Publisher state_pub = nh.advertise<std_msgs::Float32MultiArray>("camera_pose", 1);
    std_msgs::Float32MultiArray data_to_pub;

    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");
    getopt_add_int(getopt, 'c', "camera", "0", "Set camera device number");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    int camera_num = getopt_get_int(getopt, "camera");
    VideoCapture cap(camera_num);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = target_tag_create();
    tf->black_border = getopt_get_int(getopt, "border");

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    Mat frame, gray;

    ///add some parameters
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(-0.071, -0.084, 0));
    objectPoints.push_back(cv::Point3f(0.071, -0.084, 0));
    objectPoints.push_back(cv::Point3f(0.071, 0.084, 0));
    objectPoints.push_back(cv::Point3f(-0.071, 0.084, 0));
    std::vector<cv::Point2f> imagePoints;
    Mat cameraParam = (Mat_<double>(3,3) << 693.509569, 0.000000, 331.329297, 0.000000, 695.445293, 229.080487, 0.000000, 0.000000, 1.000000);
    Mat distCoeffs = (Mat_<double>(5,1) << 0.102018, -0.376755, 0.005830, 0.005656, 0.000000);
    Mat rvec, tvec; //save output
    Mat rotationMat;
    Mat cam2world = (Mat_<double>(3,3) << 1,0,0,0,-1,0,0,0,-1);

    while (ros::ok()) {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        data_to_pub.data.clear();

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);
        //cout << zarray_size(detections) << " tags detected" << endl;

        ///if no detected, return (-1,-1,-1)
        if ( 0 == zarray_size(detections)){
            data_to_pub.data.push_back(-1);
            data_to_pub.data.push_back(-1);
            data_to_pub.data.push_back(-1);
        }

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            ///calculate rotation and translation based on opencv
            imagePoints.clear();
            imagePoints.push_back(cv::Point2f(det->p[0][0], det->p[0][1]));
            imagePoints.push_back(cv::Point2f(det->p[1][0], det->p[1][1]));
            imagePoints.push_back(cv::Point2f(det->p[2][0], det->p[2][1]));
            imagePoints.push_back(cv::Point2f(det->p[3][0], det->p[3][1]));

            cv::solvePnP(objectPoints, imagePoints, cameraParam, distCoeffs, rvec, tvec);
            //cout << "translation vector" << rvec << endl;
            cv::Rodrigues(rvec, rotationMat); //get 3x3 rotation matrix
            rotationMat = cam2world * rotationMat;
            cv::Rodrigues(rotationMat, rvec); //recover to 3x1 vector
            //cout << "rotation vector" << rvec << endl;

            data_to_pub.data.push_back(tvec.at<double>(0)*100); //in cm
            data_to_pub.data.push_back(tvec.at<double>(2)*100); //in cm
            data_to_pub.data.push_back(rvec.at<double>(1)/3.1415*180); //yaw, in degree, around y
            ///////////////////////////////////

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        zarray_destroy(detections);
        state_pub.publish(data_to_pub);

        imshow("Tag Detections", frame);
        if (waitKey(30) >= 0)
            break;
    }

    apriltag_detector_destroy(td);

    target_tag_destroy(tf);
    getopt_destroy(getopt);

    return 0;
}
