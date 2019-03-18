//
// Created by controller on 9/7/17.
//
#include "gps_compass_correction/gps_compass_correction.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_correction_node");

    GpsCompassCorrection correction;
    ros::spin();
    ros::waitForShutdown();

    return 0;
}