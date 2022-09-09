//
// Created by sumin on 22. 9. 9.
//
#include "data_transfer/data_transfer.h"

using namespace std;
static inline void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data);

Data_Transfer::Data_Transfer()
:nh(""), pnh("")
{
    sub_BBOX = nh.subscribe("/darknet_ros/bounding_boxes",1,&Data_Transfer::obj_bbox_CB,this);
    sub_CNT = nh.subscribe("/darknet_ros/found_object", 1, &Data_Transfer::obj_cnt_CB, this);

    pub_TL = nh.advertise<data_transfer_msg::traffic_light>("/data_transfer/traffic_light",1);
    pub_DS = nh.advertise<data_transfer_msg::delivery_state>("/data_transfer/delivery_state",1);
    pub_DM = nh.advertise<data_transfer_msg::delivery_mission>("/data_transfer/delivery_mission", 1);

    cycle = 0;
    now_time = ros::Time::now().toSec();
    prev_time = now_time;

    delivery_init = true;

    tl = Eigen::MatrixXd::Zero(10, 1);
    dl = Eigen::MatrixXd::Zero(20,1);

    traffic_cnt = 0;
    delivery_cnt = 0;

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
    if (cycle %6==0) {
        for (auto i: bbox_vec) {
            if (i.id < 5) {
                Traffic_Light(i.id);
            } else {

            }
        }
    }
    if (cycle % 3 ==0){
        for(auto i:bbox_vec){
            if(i.id>=5 ){
                Delivery(i.id);
                break;
            }
        }
    }
    cycle +=1;
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

        delivery_state = "none";
        delivery.Now_state = delivery_state;
        pub_DS.publish(delivery);

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

void Data_Transfer::Delivery(int id) {
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(1,1);
    data << id;
    updateSample(dl, data);
    delivery_cnt +=1;

    if (delivery_cnt >= 20){
        delivery_state = return_ID(mode(dl,20));
        delivery.Now_state = delivery_state;
        if (delivery_init == true){
            delivery.Pick_up= delivery_state;
            delivery.Delivery= to_go(delivery_state);
            delivery_init = false;
        }
        pub_DS.publish(delivery);
    }
    else {
        ROS_INFO("Waiting for Delivery_signal");
    }
}

string Data_Transfer::to_go(std::string value) {
    if (value =="A1"){
        return "B1";
    }
    else if (value == "A2"){
        return "B2";
    }
    else if (value == "A3"){
        return "B3";
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

string Data_Transfer::return_ID(int value) {
    switch(value){
        case 0:
            return "G";
        case 1:
            return "R";
        case 2:
            return "LG";
        case 3:
            return "LR";
        case 4:
            return "Y";
        case 5:
            return "A1";
        case 6:
            return "A2";
        case 7:
            return "A3";
        case 8:
            return "B1";
        case 9:
            return "B2";
        case 10:
            return "B3";
    }
}