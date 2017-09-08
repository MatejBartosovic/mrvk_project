//
// Created by controller on 9/7/17.
//

#include "gps_compas_correction/GpsCompasCorrection.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

GpsCompasCorrection::GpsCompasCorrection() : n("~"),correctionTransform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)){

    double tfRate, correctioninterval;
    std::string blockMovementServerName, clearCostMapServerName, map;

    n.param<double>("tf_rate", tfRate, 20);
    n.param<double>("correction_interval", correctioninterval, 30);
    n.param<std::string>("parrent_frame", parrentFrame, "world");
    n.param<std::string>("child_frame", childFrame, "map");
    n.param<std::string>("target_frame", targetFrame, "base_link");
    n.param<std::string>("block_movement_server_name",blockMovementServerName,"/block_movement");
    n.param<std::string>("clear_costmap_server_name",clearCostMapServerName,"/move_base/clear_costmaps");
    n.param<std::string>("imu_topic",imuTopic,"/imu");
    n.param<std::string>("gps_topic",gpsTopic,"/gps");
    n.param<std::string>("map",gpsTopic,"");


    blockMovementClient = n.serviceClient<std_srvs::SetBool>(blockMovementServerName);
    clearCostMapClient = n.serviceClient<std_srvs::SetBool>(clearCostMapServerName);

    tfTimer = n.createTimer(ros::Duration(1/tfRate), &GpsCompasCorrection::tfTimerCallback,this);
    correctionTimer = n.createTimer(ros::Duration(correctioninterval), &GpsCompasCorrection::correctionTimerCallback,this);
    osm_planner::Parser parser;
    parser.setNewMap(map);
    parser.parse();
    mapOrigin = parser.getNodeByID(0);
    //osm_planner::Parser::Haversine::get
}

void GpsCompasCorrection::tfTimerCallback(const ros::TimerEvent& event){
    tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
}

void GpsCompasCorrection::correctionTimerCallback(const ros::TimerEvent& event){

    //stop robot
    if(!stopRobot())
        return;

    //get transformation
    tf::StampedTransform relativeTransform;
    try{
        listener.lookupTransform(childFrame, targetFrame, ros::Time(0), relativeTransform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        runRobot();
        return;
    }

    //get imu and gps data
    boost::shared_ptr<const sensor_msgs::Imu> imuData  = ros::topic::waitForMessage<sensor_msgs::Imu>(imuTopic, ros::Duration(1));
    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(1));
    if(!imuData || !gpsData){
        ROS_ERROR("imu or gsg data timout");
        runRobot();
        return;
    }

    //construct compads quaternion
    tf::Quaternion imuQuaternion;
    tf::quaternionMsgToTF(imuData->orientation,imuQuaternion);

    //construct gps translation
    tf::Vector3 gpsTranslation(osm_planner::Parser::Haversine::getCoordinateX(mapOrigin,*gpsData),osm_planner::Parser::Haversine::getCoordinateY(mapOrigin,*gpsData),0); //todo misov prepocet dorobit

    //absolute orientation
    tf::Transform absolutTransform(imuQuaternion,gpsTranslation);

    //compute correction
    correctionTransform = absolutTransform * relativeTransform.inverse();

    //publish correction
    tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));

    //run robot
    runRobot();
}

bool GpsCompasCorrection::stopRobot(){
    tfTimer.stop();
    std_srvs::SetBool srv;
    srv.request.data = true;
    if(!blockMovementClient.call(srv)|| !srv.response.success){
        ROS_ERROR("Unable to stop robot");
        tfTimer.start();
        return false;
    }
    std_srvs::Empty emptySrv;
    if(!clearCostMapClient.call(emptySrv))
        ROS_ERROR("Unable to clear costmaps");
    sleep(2);
    return  true;
}

void GpsCompasCorrection::runRobot(){
    tfTimer.start();
    std_srvs::SetBool srv;
    srv.request.data = false;
    if(!blockMovementClient.call(srv)|| !srv.response.success){
        ROS_ERROR("Unable to run robot");
        return;
    }
}
