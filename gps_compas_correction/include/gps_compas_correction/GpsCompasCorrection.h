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
#include <osm_planner/computeBearing.h>
#include <mutex>


class GpsCompasCorrection {
public:
    GpsCompasCorrection();
    void init();

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
    ros::ServiceServer computeBearing;

    ros::Publisher gps_odom_pub;

    std::mutex transformationMutex;

    bool computeBearingCallback(osm_planner::computeBearing::Request &req, osm_planner::computeBearing::Response &res);

    /*template<class N> void publishOdometry(N gpsPose, tf::Quaternion quat = tf::createQuaternionFromYaw(0)){

        //gps to odom publisher
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = childFrame;
        odom.child_frame_id = target_frame;

        odom.pose.pose.position.x = osm_planner::Parser::Haversine::getCoordinateX(mapOrigin, gpsPose);
        odom.pose.pose.position.y = osm_planner::Parser::Haversine::getCoordinateY(mapOrigin, gpsPose);
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        gps_odom_pub.publish(odom);
    }*/

    template<class N>
    void sendTransform(N gpsPose, tf::Quaternion quat, bool waitForTransform = false){

        //get transformation
        tf::StampedTransform relativeTransform;
        try{

            if (waitForTransform)
                listener.waitForTransform(childFrame, targetFrame, ros::Time(0), ros::Duration(1));

            listener.lookupTransform(childFrame, targetFrame, ros::Time(0), relativeTransform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            runRobot();
            return;
        }

        //construct gps translation
        tf::Vector3 gpsTranslation(osm_planner::Parser::Haversine::getCoordinateX(mapOrigin,gpsPose),osm_planner::Parser::Haversine::getCoordinateY(mapOrigin, gpsPose),0); //todo misov prepocet dorobit

        //absolute orientation
        tf::Transform absolutTransform(quat, gpsTranslation);

        //compute correction
       // std::lock_guard(transformationMutex);
        transformationMutex.lock();
        correctionTransform = absolutTransform * relativeTransform.inverse();

        //publish correction
        tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
        transformationMutex.unlock();
    }
};


#endif //PROJECT_GPSCOMPASCORRECTION_H
