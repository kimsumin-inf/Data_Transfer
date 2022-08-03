#include "Data_Transfer/Data_Transfer.h"

using namespace std;

int main(int argc, char** argv){
    ros::init(argc,argv, "data_transfer");
    Data_Transfer df;
    ros::spin();
    return 0;
}