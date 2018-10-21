//
// Created by matejko on 14.10.2018.
//
#include <ros/ros.h>
#include <adis16488/SerialCommunicationInterface.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "adis16488");
    ros::NodeHandle n;
    try{
        Adis16488::SerialCommunicationInterface ci;
        std::vector<uint8_t> reg = {0x00,0x7E, 0x7E};
        std::vector<uint16_t> data;
        ci.redRegisters(reg,data);
    }
    catch (serial::SerialException &e){
        ROS_ERROR("%s",e.what());
    }

    ros::spin();
}
