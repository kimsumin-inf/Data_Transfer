#include "Data_Transfer/Data_Transfer.h"

using namespace std;
using namespace cv;

/*
 * funcition initialize;
 */
static inline void updateSample(Eigen::MatrixXd &sample, Eigen::MatrixXd &data);
int mode(Eigen::MatrixXd data, int size);
string return_ID(int value);
string to_go(string value);
void return_grayscale (Mat &frame);
int return_center_point(int x_min, int x_max);


Data::Data()
: nh("")
{
    subCOUNT = nh.subscribe("/darknet_ros/found_object" ,1,&Data::obj_cnt_CB, this);
    subBBOX = nh.subscribe("/darknet_ros/bounding_boxes" ,1,&Data::obj_bbox_CB, this);
    subCAM = nh.subscribe("/camera_front/image_raw", 1, &Data::cam_CB, this);
    pubRESULT = nh.advertise<data_transfer_msg::data_transfer>("/Data_Transfer",1);

    count =0;
    size = 15;
    traffic_count=0;
    delivery_count=0;
    delivery_Init = true;
    traffic_light = Eigen::MatrixXd::Zero(size,1);
    delivery = Eigen::MatrixXd::Zero(size,1);

    now_traffic_light_state = "none";
    init_delivery_state = "none";
    now_delivery_state = "none";
    definition_Delivery = "none";
    stop_line_ignore = true;
    stop_line_state = false;
}

void Data::obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr &msg) {
    cnt = *msg;
    count = cnt.count;
}

void Data::obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg) {
    bbox = *msg;

    vec.clear();

    for (auto i : bbox.bounding_boxes){
        id_bbox tmp;
        tmp.id = i.id;
        tmp.center = return_center_point(i.xmin, i.xmax);
        vec.push_back(tmp);
    }
    sort(vec.begin(),vec.end(), compare);
    for (auto i : vec){
        if (i.id <5){
            Traffic_light(i.id);
        }
    }
    try{
        for (auto i : vec){
            if (i.id >=5){
                Delivery(i.id);
                break;
            }
        }
    }
    catch (int expn){
        cout << expn <<endl;
    }


}



void Data::Traffic_light(int id)  {
    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(1,1);
    data << id;
    data_transfer_msg::data_transfer result;
    updateSample(traffic_light,data);

    if (traffic_count >size){
        now_traffic_light_state= return_ID(mode(traffic_light,size));
        if ((now_traffic_light_state=="R" || now_traffic_light_state=="LR" || now_traffic_light_state =="Y")){
            stop_line_ignore = false;
            pub();
        }
        else {
            stop_line_ignore = true;
            pub();
        }
    }
    else{
        ROS_INFO("Waiting for Traffic");

    }
    traffic_count+=1;


}
void Data::Delivery(int id) {
    data_transfer_msg::data_transfer result;

    Eigen::MatrixXd data = Eigen::MatrixXd::Zero(1,1);
    data<< id;
    updateSample(delivery, data);
    if (delivery_count > size){
        now_delivery_state = return_ID(mode(delivery,size));
        result.detect_Delivery = now_delivery_state;
        if (delivery_Init==true){
            init_delivery_state = now_delivery_state;
            definition_Delivery= to_go(init_delivery_state);
            delivery_Init = false;
            pub();
        }
        pub();
    }

    delivery_count+=1;
}
void Data::pub(){
    printf("\033[2J");
    printf("\033[1;1H");
    ROS_INFO("traffic_light_state: %s", now_traffic_light_state.c_str());
    ROS_INFO("stop_line_ignore: %s", stop_line_ignore ? "true" : "false");
    ROS_INFO("init_Delivery: %s", init_delivery_state.c_str());
    ROS_INFO("now_detect_Delivery: %s", now_delivery_state.c_str());
    ROS_INFO("definition_Delivery: %s", definition_Delivery.c_str());

    data_transfer_msg::data_transfer result;
    result.traffic_light = now_traffic_light_state;
    result.stop_line_ignore = stop_line_ignore;
    result.init_Delivery = init_delivery_state;
    result.detect_Delivery = now_delivery_state;
    result.definition_Delivery = definition_Delivery;
    result.stop_line_state = stop_line_state;
    result.brake = 0;
    pubRESULT.publish(result);
}
void Data::cam_CB(const sensor_msgs::Image::ConstPtr &msg) {
    ROS_INFO("cam subscirbed : (%d , %d)", msg->width, msg->height);

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        base_frame = cv_ptr->image;
    } catch(cv_bridge::Exception &e){
        ROS_ERROR("Error to Convert");
        return;
    }
    return_grayscale(base_frame);
    imshow("front_cam",base_frame);
    waitKey(1);
}
void return_grayscale (Mat& frame){
    cvtColor(frame,frame,COLOR_BGR2GRAY);
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

int mode(Eigen::MatrixXd data, int size) {
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
string to_go(string value){
    if (value == "A1"){
        return "B1";
    }
    else if  (value == "A2"){
        return "B2";
    }
    else if (value=="A3"){
        return "B3";
    }
}
int return_center_point(int x_min, int x_max){
    return (x_min + x_max) /2;
}
bool Data::compare(id_bbox a, id_bbox b){
    return a.center > b.center;
}