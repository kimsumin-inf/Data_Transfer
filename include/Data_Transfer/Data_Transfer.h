
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

class Data_Transfer{
private:
    void obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);
    void obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

    ros::NodeHandle nh;

    ros::Subscriber subCOUNT;
    ros::Subscriber subBBOX;
public:
    Data_Transfer();
};