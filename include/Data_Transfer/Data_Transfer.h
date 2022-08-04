#pragma Once
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <string>
#include <vector>
#include <cmath>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include "Data_Transfer/data_transfer.h"

class Data{
private:
    void obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);
    void obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void Traffic_light(int id);
    void Delivery(int id);


    ros::NodeHandle nh;

    ros::Subscriber subCOUNT;
    ros::Subscriber subBBOX;

    ros::Publisher pubRESULT;

    int count;
    int size;

    int traffic_count;
    int delivery_count;

    std::string now_traffic_light_state;
    std::string init_delivery_state;
    std::string now_delivery_state;

    darknet_ros_msgs::ObjectCount cnt;
    darknet_ros_msgs::BoundingBoxes bbox;

    Eigen::MatrixXd traffic_light;
    Eigen::MatrixXd delivery;
    Data_Transfer::data_transfer result;




public:
    Data();
};