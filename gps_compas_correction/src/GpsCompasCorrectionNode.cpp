//
// Created by controller on 9/7/17.
//
#include <ros/ros.h>
#include "gps_compas_correction/GpsCompasCorrection.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_correction_node");

    GpsCompasCorrection correction;

    ros::spin();
    ros::shutdown();

    return 0;
}