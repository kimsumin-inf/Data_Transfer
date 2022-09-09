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


typedef struct id_bbox{
  int id;
  int center;
};

class Data_Transfer{
private:
    void obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);

    void Traffic_Light(int id);
    void Delivery(int id);

    int return_center(int x_min, int x_max);
    static bool compare(id_bbox a, id_bbox b);
    int mode(Eigen::MatrixXd data, int size);
    std::string return_ID(int value);
    std::string to_go(std::string value);

    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber sub_BBOX;
    ros::Subscriber sub_CNT;

    ros::Publisher pub_TL;
    ros::Publisher pub_DS;
    ros::Publisher pub_DM;

    darknet_ros_msgs::BoundingBoxes BBOX;
    darknet_ros_msgs::ObjectCount  CNT;

    std::string traffic_light_state;
    std::string delivery_state;
    std::string mission_state;

    data_transfer_msg::traffic_light traffic_light;
    data_transfer_msg::delivery_state delivery;
    data_transfer_msg::delivery_mission mission;

    std::vector<id_bbox> bbox_vec;

    bool delivery_init;

    int cycle;

    double prev_time;
    double now_time;

    Eigen::MatrixXd tl;
    Eigen::MatrixXd dl;

    int traffic_cnt;
    int delivery_cnt;
public:
    Data_Transfer();

};


#endif //SRC_DATA_TRANSFER_H
