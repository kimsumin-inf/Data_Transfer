//
// Created by sumin on 22. 9. 9.
//
#include "data_transfer/data_transfer.h"

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "data_transfer");
    Data_Transfer DT;
    ros::spin();
    return 0;
}