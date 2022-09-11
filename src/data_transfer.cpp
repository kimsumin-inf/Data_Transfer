//
// Created by sumin on 22. 9. 9.
//
#include "data_transfer/data_transfer.h"

using namespace std;
static inline void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data);

Data_Transfer::Data_Transfer()
:nh(""), pnh(""), set_hz(10)
{
    sub_BBOX = nh.subscribe("/darknet_ros/bounding_boxes",1,&Data_Transfer::obj_bbox_CB,this);
    sub_CNT = nh.subscribe("/darknet_ros/found_object", 1, &Data_Transfer::obj_cnt_CB, this);

    pub_TL = nh.advertise<data_transfer_msg::traffic_light>("/data_transfer/traffic_light",1);

    now_time = ros::Time::now().toSec();
    prev_time = now_time;

    tl = Eigen::MatrixXd::Zero(10, 1);
    dl = Eigen::MatrixXd::Zero(10,1);

    traffic_cnt = 0;
}

void Data_Transfer::obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    BBOX = *msg;
    bbox_vec.clear();
    for (auto i : BBOX.bounding_boxes){
        id_bbox tmp;
        tmp.id = i.id;
        tmp.center  = return_center(i.xmin, i.xmax);
        bbox_vec.push_back(tmp);
    }
    sort(bbox_vec.begin(), bbox_vec.end(), compare);

    for (auto i: bbox_vec) {
        if (i.id < 5) {
            Traffic_Light(i.id);
        }
    }

    set_hz.sleep();
}

void Data_Transfer::obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
    CNT = *msg;
    printf("\033[2J");
    printf("\033[1;1H");
    now_time = ros::Time::now().toSec();
    if (CNT.count != 0){
        prev_time = now_time;
    }
    if (now_time - prev_time > 5){
        ROS_INFO("Initialize state because nothing is detected for 5 seconds.");
        traffic_light_state ="none";
        traffic_light.Traffic_light = traffic_light_state;
        pub_TL.publish(traffic_light);

    }
}
void Data_Transfer::Traffic_Light(int id) {
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(1,1);
    data<< id;
    updateSample(tl ,data);
    traffic_cnt +=1;
    if (traffic_cnt >= 10){
        traffic_light_state = return_ID(mode(tl,10));
        traffic_light.Traffic_light = traffic_light_state;
        pub_TL.publish(traffic_light);
    }
    else {
        ROS_INFO("Waiting for Traffic light");
    }
}

int Data_Transfer::return_center(int x_min, int x_max){
    return (x_min + x_max) /2;
}

bool Data_Transfer::compare(id_bbox a, id_bbox b) {
    return a.center> b.center;
}
static inline void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data)
{
    if (data.cols() != sample.cols() || data.rows() != 1)
    {
        ROS_WARN("invalid data for sample matrix");
        return;
    }
    Eigen::MatrixXd temp = sample.block(1,0,sample.rows() - 1, sample.cols());
    sample.block(0,0,sample.rows() - 1, sample.cols()) = temp;
    sample.block(sample.rows()- 1, 0, 1, sample.cols()) = data;
}

int Data_Transfer::mode(Eigen::MatrixXd data, int size) {
    int class_size = 11;
    vector<int> tmp(class_size, 0);
    for (int i = 0; i < size; i++) {
        tmp[data(i, 0)] += 1;
    }
    int max = tmp[0];
    int max_index = 0;
    for (int i = 0; i < class_size; i++) {
        if (max < tmp[i]) {
            max = tmp[i];
            max_index = i;
        }
    }

    return max_index;
}
