#include "lidar_based_method/hokuyo_process.h"
#define LSD
hokuyo::hokuyo(ros::NodeHandle& nh, ros::NodeHandlePtr node_ptr):
    debug(true), show(true), debug_raw(false),
    lidar_img_height(250), lidar_img_width(250),
    box_left_right(70), box_top_bottom(240),
    sobel_kernal_size(3), sobel_thres_value(600),
    min_line_point_num(10), min_x_dis(3.0),ignore_num(3)
{
    node_ = node_ptr;
    this->InitParameters(node_);
    //subscribe lidar data
    this->lidar_proc_sub = node_->subscribe("/scan", 1, &hokuyo::ScanCallback, this);
    this->block_pub = node_->advertise<std_msgs::Float32MultiArray>("/block_single_location",1);
}


void hokuyo::InitParameters(ros::NodeHandlePtr node_para)
{
    // Deron
    node_para->param("debug", debug, true);
    node_para->param("debug_raw", debug_raw, false);
    node_para->param("show", show, true);

    node_para->param("sobel_kernal_size", sobel_kernal_size, 3);
    node_para->param("sobel_thres_value", sobel_thres_value, 600);
    node_para->param("min_line_point_num", min_line_point_num, 10);

    node_para->param("lidar_img_height", lidar_img_height, 250);
    node_para->param("lidar_img_width", lidar_img_width, 250);
    node_para->param("box_left_right", box_left_right, 120);
    node_para->param("box_top_bottom", box_top_bottom, 240);
    node_para->param("min_x_dis", min_x_dis, 3.0);
    node_para->param("ignore_num", ignore_num, 3);

}


// find nearest one in left-right direction
bool cmp_by_y(cv::Point2f i, cv::Point2f j){
    return i.y <= j.y;
}

// find nearest one in up-bot direction
bool cmp_by_x(cv::Point2f i, cv::Point2f j){
    return i.x <= j.x;
}

bool cmp_by_y_abs(cv::Point2f i, cv::Point2f j){
    return fabs(i.y) <= fabs(j.y);
}

bool cmp_by_d_abs(cv::Point2f i, cv::Point2f j){
    float i_d = i.x * i.x  + i.y * i.y * 3;
    float j_d = j.x * j.x  + j.y * j.y * 3;
    return i_d <= j_d;
}

// find nearest one in up-bot direction
bool cmp_by_x_abs(cv::Point2f i, cv::Point2f j){
    return fabs(i.x) <= fabs(j.x);
}

cv::Point2f sum_point2f(cv::Point2f i, cv::Point2f j){
    return i+j;
}

void cout_vector(std::vector<cv::Point2f>& coordinates){
    for(uint j=0; j < coordinates.size(); j++){
        ROS_INFO("the vector data is: %f", coordinates[j].x);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void hokuyo::ScanCallback(const sensor_msgs::LaserScan &scan)
{

    double time1 = static_cast<double>( cv::getTickCount());
    /// scam data check
    if(scan.ranges.empty()){
        ROS_WARN("Empty lidar data!");
        return;
    }
    if(debug){ ROS_INFO("!!NEW FRAME!!");}


    /// 1. generate a lidar image and detection vector
    cv::Mat orign_img = cv::Mat::zeros(this->lidar_img_height, this->lidar_img_width, CV_8UC1);
    std::vector<cv::Point2f> coordinate;
    std::vector<cv::Point2f> blind_cords;

    int p_i = 0; // count lidar point
    for (std::vector<float>::const_iterator iter = scan.ranges.begin(); iter != scan.ranges.end(); iter++){
        float rad = scan.angle_min + p_i * scan.angle_increment;
        p_i++;
        float x = (*iter) * cos(rad) * 100;
        float y = (*iter) * sin(rad) * 100;
        if(!(finite(x) > 0 && finite(y) > 0)){ continue;}
        if(!(fabs(x) < box_top_bottom && fabs(y) < box_left_right)){continue;} // box filter
        coordinate.push_back(cv::Point2f(x, y));

        // for close detection
        if(fabs(x) < 0.4 && fabs(y) < 0.2){
            blind_cords.push_back(cv::Point2f(x, y));
        }

        // liadr image
        int img_x = int(round(x));
        int img_y = int(box_left_right - round(y));
        if(img_x >= lidar_img_width || img_y >= lidar_img_height || img_y < 0 || img_x < 0){
            ROS_ERROR("lidar image generation: out of range (%f, %f)", x, y);
            continue;
        }
        orign_img.at<uchar>(img_y, img_x) = 255;
    } // finish generate img and vector

    if(show){
        cv::Point2f src_center(orign_img.cols/2.0F, orign_img.rows/2.0F);
        cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, 90, 1.0);
        cv::Mat dst_img;
        cv::warpAffine(orign_img, dst_img, rot_mat, cv::Size(orign_img.rows, orign_img.cols));
        cv::imshow("lidar_img", dst_img); cv::waitKey(1);
    }


    /// 2. detected all the possible lines
    std::vector<cv::Point2f> median_points;
    cv::Point2f close_median(0.0, 0.0);
    int line_num  = this->lsd_detector(coordinate, median_points, scan.angle_increment);
    int close_number = this->blind_detector(blind_cords, close_median);

    if(show){
        cv::Mat draw = cv::Mat::zeros(orign_img.size(), CV_8UC3);
        for(uint i = 0; i < this->lineEndpoints_.size(); i++){
            int x1 = int(this->lineEndpoints_[i][0]);
            int y1 = int(this->lineEndpoints_[i][1]);
            int x2 = int(this->lineEndpoints_[i][2]);
            int y2 = int(this->lineEndpoints_[i][3]);
            cv::line(draw, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 255), 2);
            if(close_number != -1){
                cv::circle(draw, close_median, 5, cv::Scalar(255, 0, 0), 2);
            }
        }
        cv::Point2f src_center(draw.cols/2.0F, draw.rows/2.0F);
        cv::Mat rot_mat = cv::getRotationMatrix2D(src_center, 90, 1.0);
        cv::Mat dst_img;
        cv::warpAffine(draw, dst_img, rot_mat, cv::Size(draw.rows, draw.cols));
        cv::imshow("lines", dst_img);
        cv::waitKey(1); // 1ms
    }

    /// 3. get the nearest target
    std_msgs::Float32MultiArray data_to_pub;
    std::vector<cv::Point2f> median_points_temp;
    if(median_points.empty()){
        median_points_temp.push_back(cv::Point2f(-1.0*100, 1.0*100));
    }else if(median_points.size() == 1){
        median_points_temp.push_back(median_points[0]);
        if(debug){ROS_INFO("centermost-left-right:(y:%f, x:%f)",median_points[0].x, -median_points[0].y);}
    }else{
        std::sort(std::begin(median_points), std::end(median_points), cmp_by_d_abs);
        median_points_temp.push_back(median_points[0]);
        if(debug){ROS_INFO("centermost-distance:(y:%f, x:%f)",median_points[0].x, -median_points[0].y);}
    }
    //ROS_INFO("lidar data %f %f", median_points_temp[0].x, -median_points_temp[0].y);


    if(close_number != -1){
        median_points_temp.push_back(close_median);
    }else{
        median_points_temp.push_back(cv::Point2f(-1.0*100, 1.0*100));
    }

    data_to_pub.data.push_back(-median_points_temp[0].y / 100);
    data_to_pub.data.push_back(median_points_temp[0].x / 100);
    data_to_pub.data.push_back(-median_points_temp[1].y / 100);
    data_to_pub.data.push_back(median_points_temp[1].x / 100);

    double time2 = (static_cast<double>(cv::getTickCount()) - time1)/cv::getTickFrequency();
    if(debug){ROS_WARN("The time is: %f", time2);}
    block_pub.publish(data_to_pub);
}


int hokuyo::blind_detector(std::vector<cv::Point2f> &raw_data, cv::Point2f& median_points)
{
    if(raw_data.size() < 10){
        median_points = cv::Point2f(0.0, 0.0);
        return -1;
    }
    std::vector<cv::Point2f> coordinates;
    std::sort(std::begin(raw_data), std::end(raw_data), cmp_by_y_abs);
    for(uint i = 0; i < raw_data.size(); i++){
        float parent_y = fabs(raw_data[i].y);
        if(parent_y < 10){
            coordinates.push_back(cv::Point2f(raw_data[i].x, raw_data[i].y));
        }
    }
    if(coordinates.size() < 5){
        median_points = cv::Point2f(0.0, 0.0);
        return -1;
    }
    median_points = std::accumulate(std::begin(coordinates),
                                    std::end(coordinates),
                                    cv::Point2f(0, 0), sum_point2f);
    median_points.x = median_points.x / coordinates.size();
    median_points.y = median_points.y / coordinates.size();

    return 1;

}


int hokuyo::lsd_detector(std::vector<cv::Point2f>& raw_data,
                         std::vector<cv::Point2f>& median_points,
                         float angle)
{
    this->lineEndpoints_.clear();
    median_points.clear();
    if(raw_data.empty()){ return -1;}
    bool inter_count = false;
    float ratio = tan(angle);
    std::vector<cv::Point2f> coordinates;
    coordinates.push_back(raw_data[0]);
    for(uint i = 1; i < raw_data.size()-1; i++){
        // pick up parent point
        float parent_x = coordinates[coordinates.size()-1].x;
        float parent_y = coordinates[coordinates.size()-1].y;

        // allow at most [ignore_num] consecutive points lost
        float y_interval = fabs(parent_x) * ratio * ignore_num;
        if(debug_raw){ ROS_WARN("y_interval:%f", y_interval);}

        // filter abnormal data of child point
        float child_x = (raw_data[i-1].x+raw_data[i+1].x)*0.2 + raw_data[i].x*0.6;
        float child_y = (raw_data[i-1].y+raw_data[i+1].y)*0.1 + raw_data[i].y*0.8;
        if(fabs(child_x - raw_data[i-1].x) > 10 && fabs(child_x - raw_data[i+1].x) > 10){
            raw_data[i].x = (raw_data[i-1].x+raw_data[i+1].x)*0.5;
        }

        /// 1. count and enqueue valid data
        if(fabs(parent_x - child_x) < min_x_dis && fabs(parent_y - child_y) < y_interval){
            coordinates.push_back(raw_data[i]);
            if(i == raw_data.size()-2){inter_count = true;}
            else { inter_count = false;}
        }else{
            inter_count = true;
        }

        /// 2. once interupted or come to the end, do processing
        if(inter_count && coordinates.size() < min_line_point_num){
            coordinates.clear();
            coordinates.push_back(raw_data[i]);
            inter_count = false;
        }else if(inter_count && coordinates.size() >= min_line_point_num){
            float line_len = fabs(coordinates[coordinates.size()-1].y - coordinates[0].y); // realiable?
            if( 15 < line_len && line_len < 25 ){ // loose
                // remove larger point int x direction
                std::sort(std::begin(coordinates), std::end(coordinates), cmp_by_x);
                if(debug_raw){cout_vector(coordinates);}

                // get illegal nums
                uint illegal_count = 0;
                for(int i_remove = 0; i_remove < 3; i_remove++){
                    float larger = coordinates[coordinates.size()-1-i_remove].x;
                    if(fabs(larger-coordinates[0].x) > min_x_dis){illegal_count++;}
                }
                // remove illegal nums
                for(int i_remove = 0; i_remove < illegal_count; i_remove++){
                    coordinates.pop_back();
                }
                if(debug_raw){cout_vector(coordinates);}

                // recover point order in left-right axies
                std::sort(std::begin(coordinates), std::end(coordinates), cmp_by_y);
                float pro_line_len = fabs(coordinates[coordinates.size()-1].y - coordinates[0].y);
                float pro_line_wid = fabs(coordinates[coordinates.size()-1].x - coordinates[0].x);
                if(pro_line_wid/pro_line_len > 1){
                    coordinates.clear();
                    coordinates.push_back(raw_data[i]);
                    inter_count =false;
                }else{
                    // get median point and end point
                    std::array<float, 4> lineEndP;
                    lineEndP[0] = coordinates[0].x;
                    lineEndP[1] = box_left_right - round(coordinates[0].y);
                    lineEndP[2] = coordinates[coordinates.size()-1].x;
                    lineEndP[3] = box_left_right - round(coordinates[coordinates.size()-1].y);
                    this->lineEndpoints_.push_back(lineEndP);
                    cv::Point2f average = std::accumulate(std::begin(coordinates),
                                                          std::end(coordinates),
                                                          cv::Point2f(0, 0), sum_point2f);
                    average.x = average.x / coordinates.size();
                    average.y = average.y / coordinates.size();
                    median_points.push_back(average);
                    if(debug){ROS_INFO("median point is(%f, %f)", average.x, average.y);}
                    coordinates.clear();
                    coordinates.push_back(raw_data[i]);
                    inter_count =false;
                }
            }else{
                coordinates.clear();
                coordinates.push_back(raw_data[i]);
                inter_count =false;
            }
        } /// 2. finished

    }// for loop
    if(debug){ROS_WARN("line_num:%d", this->lineEndpoints_.size());}
    if(this->lineEndpoints_.empty()){return -1;}
    else{return this->lineEndpoints_.size();}
}





