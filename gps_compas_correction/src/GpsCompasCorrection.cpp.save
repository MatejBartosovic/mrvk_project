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
    std::string cmd_vel_topic_name;
    int settingOrigin;
    double latitude, longitude;

    n.param<double>("tf_rate", tfRate, 20);
    n.param<double>("correction_interval", correctioninterval, 30);
    n.param<std::string>("parrent_frame", parrentFrame, "world");
    n.param<std::string>("child_frame", childFrame, "map");
    n.param<std::string>("target_frame", targetFrame, "base_link");
    n.param<std::string>("block_movement_server_name",blockMovementServerName,"/block_movement");
    n.param<std::string>("clear_costmap_server_name",clearCostMapServerName,"/move_base/clear_costmaps");
    n.param<std::string>("imu_topic",imuTopic,"/imu");
    n.param<std::string>("gps_topic",gpsTopic,"/gps");
    n.param<std::string>("cmd_vel_topic", cmd_vel_topic_name, "cmd_vel");
    n.param<double>("moving_time", wait, 2);
    n.param<double>("moving_velocity", velocity, 1);
    n.param<double>("min_quaternion_w", minQuaternionWForUpdate, 0.02);
    n.param<double>("min_distance", minDistanceForUpdate, 0.5);
    n.param<bool>("use_bearing_auto_update", useBearingAutoUpdate, false);
    n.param<int>("max_count_of_sbas_fix", max_count_of_sbas_fix, 20);
    n.param<std::string>("osm_map_path",map,"");
    n.getParam("filter_of_ways",types_of_ways);
    n.getParam("set_origin_pose", settingOrigin);
    n.param<double>("origin_latitude", latitude,0);
    n.param<double>("origin_longitude",longitude,0);


    blockMovementClient = n.serviceClient<std_srvs::SetBool>(blockMovementServerName);
    clearCostMapClient = n.serviceClient<std_srvs::SetBool>(clearCostMapServerName);

    tfTimer = n.createTimer(ros::Duration(1/tfRate), &GpsCompasCorrection::tfTimerCallback,this);
    correctionTimer = n.createTimer(ros::Duration(correctioninterval), &GpsCompasCorrection::correctionTimerCallback,this);

    computeBearing = n.advertiseService("compute_bearing", &GpsCompasCorrection::computeBearingCallback, this);
    autoComputeBearing = n.advertiseService("auto_compute_bearing", &GpsCompasCorrection::autoComputeBearingCallback, this);

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1);

    osm_planner::Parser parser;
    parser.setNewMap(map);
    parser.setTypeOfWays(types_of_ways);

    if (settingOrigin == 3) {
        parser.parse();
        //mapOrigin = parser.getNodeByID(parser.getNearestPoint(latitude, longitude));
        mapOrigin.latitude = latitude;
        mapOrigin.longitude = longitude;
    }else{
        parser.parse(true);
        mapOrigin = parser.getNodeByID(0);
        // ROS_ERROR("lat %f, lon %f", mapOrigin.latitude,mapOrigin.longitude);
    }

    firstPointAdded = false;
}

void GpsCompasCorrection::tfTimerCallback(const ros::TimerEvent& event){
    tfBroadcaster.sendTransform(tf::StampedTransform(correctionTransform, ros::Time::now(), parrentFrame, childFrame));
}

void GpsCompasCorrection::correctionTimerCallback(const ros::TimerEvent& event){

    //stop robot
    //if(!stopRobot())
    //    return;

    if (useBearingAutoUpdate){
        bearingAutoUpdate();
    } else {
        gpsCompasUpdate();
    }

    //run robot
   // runRobot();
}

void GpsCompasCorrection::gpsCompasUpdate() {

    static int bad_fix_counter = 0;

    //get imu and gps data
    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(3));
    //boost::shared_ptr<const sensor_msgs::Imu> imuData = ros::topic::waitForMessage<sensor_msgs::Imu>(imuTopic, ros::Duration(1));
    boost::shared_ptr<sensor_msgs::Imu> imuData(new sensor_msgs::Imu());


    if(!gpsData || !imuData){
        ROS_WARN("IMU or GPS data timeout");
        bad_fix_counter++;
        //  runRobot();
        return;
    }

    if(gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX || gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_FIX){
        ROS_WARN("Bad GPS data");
        //    runRobot();
        bad_fix_counter++;
        return;
    }

    if (gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_SBAS_FIX){

        if (++bad_fix_counter < max_count_of_sbas_fix)
            return;
    }

    //construct compads quaternion
    // tf::Quaternion imuQuaternion;
    // tf::quaternionMsgToTF(imuData->orientation,imuQuaternion);

    //sendTransform(*gpsData, imuQuaternion); //TODO compas

    bad_fix_counter = 0;
    sendTransform(*gpsData);
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

void GpsCompasCorrection::runRobot() {
    std_srvs::SetBool srv;
    srv.request.data = false;
    if (!blockMovementClient.call(srv) || !srv.response.success) {
        ROS_ERROR("Unable to run robot");
        return;
    }
}

void GpsCompasCorrection::init(){

        boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(0));
        //boost::shared_ptr<const sensor_msgs::Imu> imuData = ros::topic::waitForMessage<sensor_msgs::Imu>(imuTopic, ros::Duration(0)); //TODO compas
    boost::shared_ptr< sensor_msgs::Imu> imuData(new sensor_msgs::Imu());

    if(gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX){
        ROS_ERROR("Bad GPS data - status no fix");
        init();
        return;
    }

    if(!imuData || !gpsData ){
        init();
        ROS_ERROR("no imu or gps data");
        return;
    }

    tf::Quaternion imuQuaternion;
    tf::quaternionMsgToTF(imuData->orientation,imuQuaternion);
    //sendTransform(*gpsData,imuQuaternion);//TODO compas
    sendTransform(*gpsData);
    return;
}


//Compute Bearing callbacks
bool GpsCompasCorrection::computeBearingCallback(osm_planner::computeBearing::Request &req, osm_planner::computeBearing::Response &res){

    if (req.bearing != 0){

        quat.setRPY(0, 0, req.bearing);

        //Get second point from GPS
        boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(3));
        if(!gpsData || gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX || gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_FIX){
            ROS_WARN("No fixed GPS data");

            return computeBearingCallback(req, res);
        }
        sendTransform(*gpsData, quat);
        return true;

    }else {

        double angle;
        int result = addPointAndCompute(&angle);
      //  firstPointAdded = false;

        if (result < 0) {
            ROS_WARN("No fixed GPS data");
            res.message = "No fixed GPS data";

        } else if (result == 0) {
            res.message = "Added first point, please move robot forward and call service again";
            res.bearing = 0;
        } else {
            res.message = "Bearing was calculated";
            res.bearing = angle;
        }
    }
    return true;
}

bool GpsCompasCorrection::autoComputeBearingCallback(std_srvs::Trigger::Request &req,
                                                     std_srvs::Trigger::Response &res) {

    autoUpdateMutex.lock();
    int result = -1;
    double angle;
    int numberOfRead = 0;

    firstPointAdded = false;
    //Add First point
    while (result < 0){

        result = addPointAndCompute(&angle);
        numberOfRead++;

        if (numberOfRead > 5) {
            ROS_WARN("No fixed GPS data");
            res.success = false;
            res.message = "No fixed GPS data in first point";
            autoUpdateMutex.unlock();
            return true;
        }
    }

    ROS_WARN("Added first point");
    numberOfRead = 0;
    result = -1;

    //Move robot
    //Set twist message
    geometry_msgs::Twist twist;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    twist.linear.z = 0;
    twist.linear.y = 0;
    twist.linear.x = velocity;

    //set timing and publish cmd_vel
    ros::Time start = ros::Time::now();
    ros::Time current = ros::Time::now();
    ros::Rate rate(5);

    while ((current - start).toSec() < wait){

        cmd_vel_pub.publish(twist);
        rate.sleep();
        current = ros::Time::now();
    }

    //stop publishing cmd_vel
    twist.linear.x = 0;
    cmd_vel_pub.publish(twist);

    //Add second point and compute
    while (result < 0){

        result = addPointAndCompute(&angle);
        numberOfRead++;

        if (numberOfRead > 5) {
            ROS_WARN("No fixed GPS data");
            res.message = "No fixed GPS data in second point";
            res.success = false;
            autoUpdateMutex.unlock();
            return true;
        }

    }

    //Print result
    res.message = "Computed angle: " + std::to_string(angle);
    res.success = true;
    autoUpdateMutex.unlock();
    return true;
}

int GpsCompasCorrection::addPointAndCompute(double *angle) {


    static osm_planner::Parser::OSM_NODE firstPoint;
    //static bool firstPointAdded = false;

    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsData = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(3));
    if(!gpsData || gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX || gpsData->status.status == sensor_msgs::NavSatStatus::STATUS_FIX){
        ROS_WARN("No fixed GPS data");
     //   res.message = "No fixed GPS data";
        return -1;
    }

    if (!firstPointAdded){
        firstPoint.longitude = gpsData->longitude;
        firstPoint.latitude = gpsData->latitude;
       // res.message = "Added first point, please move robot forward and call service again";
       // res.bearing = 0;
        firstPointAdded  = true;
        return 0;
    } else{

        osm_planner::Parser::OSM_NODE secondPoint;
        secondPoint.longitude = gpsData->longitude;
        secondPoint.latitude = gpsData->latitude;
        double calculatedAngle = -osm_planner::Parser::Haversine::getBearing(firstPoint, secondPoint);
        //res.message = "Bearing was calculated";
        firstPointAdded = false;
        //tf::Quaternion q;
        quat.setRPY(0, 0, calculatedAngle);
        sendTransform(secondPoint, quat);
        *angle = calculatedAngle;
        //res.bearing = angle;
        return 1;
    }
}

bool GpsCompasCorrection::getTransformQuaternion(tf::Quaternion *quat) {

    //get transformation
    tf::StampedTransform transform;

    try{

        listener.waitForTransform(childFrame, targetFrame, ros::Time(0), ros::Duration(1));
        listener.lookupTransform(childFrame, targetFrame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("Nekorigujem polohu tf timout. %s",ex.what());
       // runRobot();
        return false;
    }

    *quat = transform.getRotation();
    return true;
}

void GpsCompasCorrection::bearingAutoUpdate() {

   if ( !autoUpdateMutex.try_lock() )
       return;

    static int bad_fix_counter = 0;

    ROS_ERROR("auto update start");
    tf::Quaternion firstRotation, secondRotation;

    firstPointAdded = false;

    //----------------------//
    //Get first point from GPS
    //----------------------//
    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsDataFirst = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(gpsTopic, ros::Duration(3));
    if(!gpsDataFirst || gpsDataFirst->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX || gpsDataFirst->status.status == sensor_msgs::NavSatStatus::STATUS_FIX){
        ROS_WARN("No fixed GPS data");
        autoUpdateMutex.unlock();
        bad_fix_counter++;
        return;
    }

    //----------------------//
    // Check gps sbas status
    //----------------------//

    if (gpsDataFirst->status.status == sensor_msgs::NavSatStatus::STATUS_SBAS_FIX){

        if (++bad_fix_counter < max_count_of_sbas_fix){
         ROS_ERROR("only sbas fix");  
	  return;
	}  else {
            bad_fix_counter = 0;
	   ROS_ERROR("correction when only sbas fix");
            sendTransform(*gpsDataFirst);
            return;
        }
    }

    //----------------------//
    //If fix is gbas - send transform
    //----------------------//
    bad_fix_counter = 0;
    sendTransform(*gpsDataFirst);

    //----------------------//
    //Get rotation on First point
    //----------------------//

    if (!getTransformQuaternion(&firstRotation)){

        ROS_ERROR("no transform received");
        bad_fix_counter++;
        autoUpdateMutex.unlock();
        return;
    }

    //----------------------//
    //Get second point from GPS
    //----------------------//

    double dist = 0;
    boost::shared_ptr<const sensor_msgs::NavSatFix> gpsDataSecond;
    static int counter = 0;

    //----------------------//
    //Wait for minimal distance for update and wait for gbas fix. Max count of meassurment is 5
    //----------------------//

    while (dist < minDistanceForUpdate) {

        //Get second point from GPS
        gpsDataSecond = ros::topic::waitForMessage<sensor_msgs::NavSatFix>(
                gpsTopic, ros::Duration(3));
        if (!gpsDataSecond || counter > 5 || !gpsDataSecond->status.status == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
            ROS_WARN("No fixed GPS data");
            counter = 0;
	    autoUpdateMutex.unlock();
            return;
        }

        counter++;
        dist = osm_planner::Parser::Haversine::getDistance(*gpsDataFirst, *gpsDataSecond);
    }
 
   counter = 0;

    //----------------------//
     //Get rotation on Second Point
    //----------------------//
     if (!getTransformQuaternion(&secondRotation)) {

         //When rotation is bad send only position
         sendTransform(*gpsDataSecond);
         ROS_ERROR("no second transform received");
         autoUpdateMutex.unlock();
         return;
     }


    //----------------------//
    //Compare Quaternions
    //----------------------//

    tf::Quaternion relativeQuaternion;
    relativeQuaternion = firstRotation * secondRotation.inverse();
    ROS_ERROR("relative rotation x %f, y %f, z %f, w %f", relativeQuaternion.x(), relativeQuaternion.y(), relativeQuaternion.z(), relativeQuaternion.w());
    double diffW = 1 - fabs(relativeQuaternion.w());

    if ( diffW > 0.1 && dist < 0.2){
        ROS_ERROR("Quaterion w diff %f", diffW);
        sendTransform(*gpsDataSecond);
        autoUpdateMutex.unlock();
        return;
    }

    //----------------------//
    //Calculating angle and send transformation
    //----------------------//

    double calculatedAngle = -osm_planner::Parser::Haversine::getBearing(*gpsDataFirst, *gpsDataSecond);
    quat.setRPY(0, 0, calculatedAngle);
    sendTransform(*gpsDataSecond, quat);
    autoUpdateMutex.unlock();
}
