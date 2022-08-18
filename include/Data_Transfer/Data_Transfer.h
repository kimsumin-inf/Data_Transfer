#pragma Once
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <data_transfer_msg/data_transfer.h>
#include <sensor_msgs/Image.h>

class Data{
private:
    struct id_bbox{
        int id;
        int center;
    };

    void obj_cnt_CB(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);
    void obj_bbox_CB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void cam_CB(const sensor_msgs::Image::ConstPtr &msg);
    void Traffic_light(int id);
    void Delivery(int id);
    static bool compare(id_bbox a, id_bbox b);

    ros::NodeHandle nh;

    ros::Subscriber subCOUNT;
    ros::Subscriber subBBOX;
    ros::Subscriber subCAM;

    ros::Publisher pubRESULT;

    int count;
    int size;

    int traffic_count;
    int delivery_count;
    bool delivery_Init;

    std::string now_traffic_light_state;
    std::string init_delivery_state;
    std::string now_delivery_state;

    darknet_ros_msgs::ObjectCount cnt;
    darknet_ros_msgs::BoundingBoxes bbox;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat base_frame;

    Eigen::MatrixXd traffic_light;
    Eigen::MatrixXd delivery;
    data_transfer_msg::data_transfer result;
    std::vector<id_bbox> vec;




public:
    Data();
};