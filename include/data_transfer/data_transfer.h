//
// Created by sumin on 22. 9. 9.
//

#ifndef SRC_DATA_TRANSFER_H
#define SRC_DATA_TRANSFER_H

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>

#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <data_transfer_msg/traffic_light.h>
#include <data_transfer_msg/delivery_state.h>
#include <data_transfer_msg/delivery_mission.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

struct id_bbox{
  int id;
  int center;
};

class Data_Transfer{
private:
    void obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);

    void Traffic_Light(int id);

    int return_center(int x_min, int x_max);
    static bool compare(id_bbox a, id_bbox b);
    int mode(Eigen::MatrixXd data, int size);

    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Rate set_hz;

    ros::Subscriber sub_BBOX;
    ros::Subscriber sub_CNT;

    ros::Publisher pub_TL;


    darknet_ros_msgs::BoundingBoxes BBOX;
    darknet_ros_msgs::ObjectCount  CNT;

    std::string traffic_light_state;

    data_transfer_msg::traffic_light traffic_light;


    std::vector<id_bbox> bbox_vec;


    double prev_time;
    double now_time;

    Eigen::MatrixXd tl;
    Eigen::MatrixXd dl;

    int traffic_cnt;

public:
    Data_Transfer();

};

#endif //SRC_DATA_TRANSFER_H
