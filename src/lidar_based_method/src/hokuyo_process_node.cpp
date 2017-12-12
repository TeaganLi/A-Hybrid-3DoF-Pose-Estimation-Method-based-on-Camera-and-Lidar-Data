#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include "lidar_based_method/hokuyo_process.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "hokuyo_proc");
    ros::NodeHandle nh;  //just for init the node
    ros::NodeHandlePtr nh_ = boost::make_shared<ros::NodeHandle>("~"); //to sub and pub topics
    hokuyo* lidar = new hokuyo(nh, nh_);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
    delete lidar;
    lidar = NULL;
}
