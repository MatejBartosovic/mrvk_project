//
// Created by controller on 9/7/17.
//

#include "gps_compas_correction/GpsCompasCorrection.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

GpsCompasCorrection::GpsCompasCorrection() : n("~"),correctionTransform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)){

    double tfRate, correctioninterval;
    std::string blockMovementServerName, clearCostMapServerName, map;
    std::vector<std::string> types_of_ways;

    n.param<double>("tf_rate", tfRate, 20);
    n.param<double>("correction_interval", correctioninterval, 30);
    n.param<std::string>("parrent_frame", parrentFrame, "world");
    n.param<std::string>("child_frame", childFrame, "map");
    n.param<std::string>("target_frame", targetFrame, "base_link");
    n.param<std::string>("block_movement_server_name",blockMovementServerName,"/block_movement");
    n.param<std::string>("clear_costmap_server_name",clearCostMapServerName,"/move_base/clear_costmaps");
    n.param<std::string>("imu_topic",imuTopic,"/imu");
    n.param<std::string>("gps_topic",gpsTopic,"/gps");
    n.param<std::string>("osm_map_path",map,"");
    n.getParam("filter_of_ways",types_of_ways);


    blockMovementClient = n.serviceClient<std_srvs::SetBool>(blockMovementServerName);
    clearCostMapClient = n.serviceClient<std_srvs::SetBool>(clearCostMapServerName);
    computeBearing = n.advertiseService("compute_bearing", &GpsCompasCorrection::computeBearingCallback, this);

    tfTimer = n.createTimer(ros::Duration(1/tfRate), &GpsCompasCorrection::tfTimerCallback,this);
    correctionTimer = n.createTimer(ros::Duration(correctioninterval), &GpsCompasCorrection::correctionTimerCallback,this);
    osm_planner::Parser parser;
    parser.setNewMap(map);
    parser.setTypeOfWays(types_of_ways);
    parser.parse(true);
    mapOrigin = parser.getNodeByID(0);
}

void GpsCompasCorrection::tfTimerCallback(const ros::TimerEvent& event){
    tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
}

void GpsCompasCorrection::correctionTimerCallback(const ros::TimerEvent& event){

    //stop robot
    if(!stopRobot())
        return;

    //get transformation
 /*   tf::StampedTransform relativeTransform;
    try{
        listener.lookupTransform(childFrame, targetFrame, ros::Time(0), relativeTransform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        runRobot();
        return;
    }*/

    //get imu and gps data
    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(3));
    //boost::shared_ptr<const sensor_msgs::Imu> imuData = ros::topic::waitForMessage<sensor_msgs::Imu>(imuTopic, ros::Duration(1));
    boost::shared_ptr<sensor_msgs::Imu> imuData(new sensor_msgs::Imu());
    imuData->orientation.x = 0;
    imuData->orientation.y = 0;
    imuData->orientation.z = 0;
    imuData->orientation.w = 1;

    if(!gpsData || !imuData){
        ROS_WARN("IMU or GPS data timeout");
        runRobot();
        return;
    }

    if(gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX){
        ROS_WARN("Bad GPS data");
        runRobot();
        return;
    }

    //construct compads quaternion
    tf::Quaternion imuQuaternion;
    tf::quaternionMsgToTF(imuData->orientation,imuQuaternion);

    sendTransform(*gpsData, imuQuaternion);
    //construct gps translation
   /* tf::Vector3 gpsTranslation(osm_planner::Parser::Haversine::getCoordinateX(mapOrigin,*gpsData),osm_planner::Parser::Haversine::getCoordinateY(mapOrigin,*gpsData),0); //todo misov prepocet dorobit

    //absolute orientation
    tf::Transform absolutTransform(imuQuaternion,gpsTranslation);

    //compute correction
    correctionTransform = absolutTransform * relativeTransform.inverse();

    //publish correction
    tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
*/
    //run robot
    runRobot();
}

bool GpsCompasCorrection::stopRobot(){
    std_srvs::SetBool srv;
    srv.request.data = true;
    if(!blockMovementClient.call(srv)|| !srv.response.success){
        ROS_ERROR("Unable to stop robot");
        return false;
    }
    std_srvs::Empty emptySrv;
   // if(!clearCostMapClient.call(emptySrv))
    //    ROS_ERROR("Unable to clear costmaps");
    sleep(2);
    return  true;
}

void GpsCompasCorrection::runRobot(){
    std_srvs::SetBool srv;
    srv.request.data = false;
    if(!blockMovementClient.call(srv)|| !srv.response.success){
        ROS_ERROR("Unable to run robot");
        return;
    }
}


bool GpsCompasCorrection::computeBearingCallback(osm_planner::computeBearing::Request &req, osm_planner::computeBearing::Response &res){

    static osm_planner::Parser::OSM_NODE firstPoint;
    static bool firstPointAdded = false;
    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(3));

    if (!firstPointAdded){
        firstPoint.longitude = gpsData->longitude;
        firstPoint.latitude = gpsData->latitude;
        res.message = "Added first point, please move robot forward and call service again";
        res.bearing = 0;
        firstPointAdded  = true;
        return true;
    } else{

        osm_planner::Parser::OSM_NODE secondPoint;
        secondPoint.longitude = gpsData->longitude;
        secondPoint.latitude = gpsData->latitude;
        double angle = osm_planner::Parser::Haversine::getBearing(firstPoint, secondPoint);
        res.message = "Bearing was calculated";
        firstPointAdded = false;
        tf::Quaternion q;
        q.setRPY(0, 0, angle);
        sendTransform(secondPoint, q, true);
        res.bearing = angle;
        return true;
    }
}
void GpsCompasCorrection::init(){

        boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(0));
        //boost::shared_ptr<const sensor_msgs::Imu> imuData = ros::topic::waitForMessage<sensor_msgs::Imu>(imuTopic, ros::Duration(0));
    boost::shared_ptr< sensor_msgs::Imu> imuData(new sensor_msgs::Imu());
    imuData->orientation.x = 0;
    imuData->orientation.y = 0;
    imuData->orientation.z = 0;
    imuData->orientation.w = 1;

    if(!imuData || !gpsData){
        init();
        ROS_ERROR("no imu or gps data");
        return;
    }
    tf::Quaternion imuQuaternion;
    tf::quaternionMsgToTF(imuData->orientation,imuQuaternion);
    sendTransform(*gpsData,imuQuaternion,true);
    return;
}
