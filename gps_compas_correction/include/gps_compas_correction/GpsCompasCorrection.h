//
// Created by controller on 9/7/17.
//

#ifndef PROJECT_GPSCOMPASCORRECTION_H
#define PROJECT_GPSCOMPASCORRECTION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <osm_planner/osm_parser.h>

class GpsCompasCorrection {
public:
    GpsCompasCorrection();
protected:
    bool stopRobot();
    void runRobot();
    osm_planner::Parser::OSM_NODE mapOrigin;
private:
    //functions
    void tfTimerCallback(const ros::TimerEvent& event);
    void correctionTimerCallback(const ros::TimerEvent& event);

    //parameters
    std::string parrentFrame,childFrame, targetFrame, imuTopic, gpsTopic;

    //ros variables
    tf::TransformBroadcaster tfBroadcaster;
    tf::TransformListener listener;
    tf::Transform correctionTransform;

    ros::Timer tfTimer, correctionTimer;
    ros::NodeHandle n;

    ros::ServiceClient blockMovementClient, clearCostMapClient;
    ros::ServiceServer correctionService; //todo dorobit
};


#endif //PROJECT_GPSCOMPASCORRECTION_H
