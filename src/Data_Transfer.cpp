#include "Data_Transfer/Data_Transfer.h"
using namespace std;

Data_Transfer::Data_Transfer()
: nh("")
{
    subCOUNT = nh.subscribe("/darknet_ros/found_object" ,1,&Data_Transfer::obj_cnt_CB, this);
    subBBOX = nh.subscribe("/darknet_ros/bounding_boxes" ,1,&Data_Transfer::obj_bbox_CB, this);

    count =0;
    traffic_light = Eigen::MatrixXd::Zero()
}

void Data_Transfer::obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
    cnt = *msg;
    count = msg->count;
}

void Data_Transfer::obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    bbox = *msg;
    for (auto i  : bbox.bounding_boxes){
        cout <<i <<endl;
    }
}