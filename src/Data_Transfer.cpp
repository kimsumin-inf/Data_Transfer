#include "Data_Transfer/Data_Transfer.h"
using namespace std;
static inline void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data);
int mode(Eigen::MatrixXd data, int size);
string return_ID(int value);

Data_Transfer::Data_Transfer()
: nh("")
{
    subCOUNT = nh.subscribe("/darknet_ros/found_object" ,1,&Data_Transfer::obj_cnt_CB, this);
    subBBOX = nh.subscribe("/darknet_ros/bounding_boxes" ,1,&Data_Transfer::obj_bbox_CB, this);

    count =0;
    size = 30;
    traffic_count=0;
    delivery_count=0;
    traffic_light = Eigen::MatrixXd::Zero(size,1);
    delivery = Eigen::MatrixXd::Zero(size,1);

}

void Data_Transfer::obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
    cnt = *msg;
    count = cnt.count;
}

void Data_Transfer::obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    bbox = *msg;
    for (auto i  : bbox.bounding_boxes){

        if (i.id < 5){
            Traffic_light(i.id);
        }
        else {
            Delivery(i.id);
        }

    }

}

void Data_Transfer::Traffic_light(int id)  {
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(1,1);
    data << id;

    updateSample(traffic_light,data);

    if (traffic_count >size){
        now_traffic_light_state= return_ID(mode(traffic_light,size));
    }
    else{
        cout <<"waiting"<<endl;
    }
    traffic_count+=1;


}

void Data_Transfer::Delivery(int id) {

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
int mode(Eigen::MatrixXd data, int size){
    int class_size =10;
    vector <int> tmp(class_size,0);
    for (int i=0;i<size;i++){
        tmp[data(i,0)] +=1;
    }
    int max=tmp[0];
    int max_index = 0;
    for (int i =0; i<class_size;i++){
        if(max<tmp[i]){
            max = tmp[i];
            max_index = i;
        }
    }

    return max_index;
}
string return_ID(int value){
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