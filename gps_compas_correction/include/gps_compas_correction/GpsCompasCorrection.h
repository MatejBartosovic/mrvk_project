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
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
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
    ros::ServiceServer autoComputeBearing;

    ros::Publisher gps_odom_pub;

    std::mutex transformationMutex;

    tf::Quaternion quat;

    //compute bearing from gps functions and datatypes
    bool computeBearingCallback(osm_planner::computeBearing::Request &req, osm_planner::computeBearing::Response &res);
    bool autoComputeBearingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    ros::Publisher cmd_vel_pub;
    double wait;
    double velocity;
    bool firstPointAdded;
    int addPointAndCompute(double *angle);


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
    void sendTransform(N gpsPose, tf::Quaternion quat){

        //get transformation
        tf::StampedTransform relativeTransform;
        try{

            listener.waitForTransform(childFrame, targetFrame, ros::Time(0), ros::Duration(1));
            listener.lookupTransform(childFrame, targetFrame, ros::Time(0), relativeTransform);
        }
        catch (tf::TransformException ex){
            ROS_WARN("Nekorigujem polohu tf timout. %s",ex.what());
            runRobot();
            return;
        }

        //construct gps translation
        tf::Vector3 gpsTranslation(osm_planner::Parser::Haversine::getCoordinateX(mapOrigin,gpsPose),osm_planner::Parser::Haversine::getCoordinateY(mapOrigin, gpsPose),0); //todo misov prepocet dorobit

        //absolute orientation
        tf::Transform absolutTransform(quat, gpsTranslation);

        //compute correction
        //std::lock_guard<std::mutex>(transformationMutex);
        transformationMutex.lock();
        correctionTransform = absolutTransform * relativeTransform.inverse();

        ROS_ERROR("service");
        printTransform(absolutTransform,"absolut");
        printTransform(relativeTransform,"relative");
        printTransform(correctionTransform,"correction");
        tf::Transform result = correctionTransform * relativeTransform;
        printTransform(result,"result");

        //publish correction
        tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
        transformationMutex.unlock();
    }

     template<class N>
    void sendTransform(N gpsPose){

        //get transformation
        tf::StampedTransform relativeTransform;
        try{
            listener.waitForTransform(childFrame, targetFrame, ros::Time(0), ros::Duration(1));
            listener.lookupTransform(childFrame, targetFrame, ros::Time(0), relativeTransform);
        }
        catch (tf::TransformException ex){
            ROS_WARN("Nekorigujem polohu tf timout. %s",ex.what());
            runRobot();
            return;
        }

         //get transformation
         tf::StampedTransform baseLinkTransform;
         try{
             listener.waitForTransform(parrentFrame, targetFrame, ros::Time(0), ros::Duration(1));
             listener.lookupTransform(parrentFrame, targetFrame, ros::Time(0), baseLinkTransform);
         }
         catch (tf::TransformException ex){
             ROS_WARN("Nekorigujem polohu tf timout. %s",ex.what());
             runRobot();
             return;
         }

        //construct gps translation
        tf::Vector3 gpsTranslation(osm_planner::Parser::Haversine::getCoordinateX(mapOrigin,gpsPose),osm_planner::Parser::Haversine::getCoordinateY(mapOrigin, gpsPose),0); //todo misov prepocet dorobit

        //absolute orientation
        tf::Transform absolutTransform(baseLinkTransform.getRotation(), gpsTranslation);

        //compute correction
         //std::lock_guard<std::mutex>(transformationMutex);
        transformationMutex.lock();
        correctionTransform = absolutTransform * relativeTransform.inverse();

         ROS_ERROR("timer");

         printTransform(absolutTransform,"absolut");
        printTransform(relativeTransform,"relative");
        printTransform(correctionTransform,"correction");
         tf::Transform result = correctionTransform * relativeTransform;
         printTransform(result,"result");

         //publish correction
        tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
        transformationMutex.unlock();
    }

    void printTransform(tf::Transform &t,std::string s){
        ROS_ERROR("%s x = %lf y = %lf z = %lf x = %lf y = %lf z = %lf w = %lf",s.c_str(),t.getOrigin().x(),t.getOrigin().y(),t.getOrigin().z(),t.getRotation().x(),t.getRotation().y(),t.getRotation().z(),t.getRotation().w());

    }
};


#endif //PROJECT_GPSCOMPASCORRECTION_H
