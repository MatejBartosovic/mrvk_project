//
// Created by controller on 9/7/17.
//

#include "gps_compass_correction/gps_compass_correction.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <osm_planner/coordinates_converters/haversine_formula.h>

GpsCompassCorrection::GpsCompassCorrection() : filter(), map_(), n_("~"), robot_(n_), corrected_transform_(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)){

    // Initialize global variables
    n_.param<bool>("use_compass", use_compass_, false);
    n_.param<std::string>("parent_frame", parent_frame_, "world");
    n_.param<std::string>("child_frame", child_frame_, "map");
    n_.param<std::string>("target_frame", target_frame_, "base_link");
    n_.param<std::string>("imu_topic",imu_topic_,"/imu");
    n_.param<std::string>("gps_topic",gps_topic_,"/gps");
    n_.param<double>("low_precision_period", low_precision_period_, 30);
    n_.param<int>("min_required_status_update", min_required_status_update_, gps_common::GPSStatus::STATUS_FIX);
    n_.param<int>("min_required_status_service", min_required_status_service_, gps_common::GPSStatus::STATUS_FIX);
    n_.param<int>("allways_allowed_status", allways_allowed_status_, gps_common::GPSStatus::STATUS_GBAS_FIX);

    // Get map origin
    setMapOrigin(n_);

    // Initialize ros subscribers, publishers and services
    gps_sub_ = n_.subscribe(gps_topic_, 1, &GpsCompassCorrection::gpsCallback, this);
    compute_bearing_service_ = n_.advertiseService("compute_bearing", &GpsCompassCorrection::computeBearingCallback, this);
    set_bearing_service_ = n_.advertiseService("set_bearing", &GpsCompassCorrection::setBearingCallback, this);
    auto_compute_bearing_service_ = n_.advertiseService("auto_compute_bearing", &GpsCompassCorrection::autoComputeBearingCallback, this);
    force_update_service_ = n_.advertiseService("force_update", &GpsCompassCorrection::forceUpdateCallback, this);

    // Create thread for broadcasting tf
    double tf_rate;
    n_.param<double>("tf_rate", tf_rate, 20);
    tf_broadcaster_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(&GpsCompassCorrection::tfBroadcasterCallback, this, tf_rate));

  //  coordinatesConverter = std::make_shared<osm_planner::coordinates_converters::HaversineFormula>();

    // Create map
    std::string map;
    std::vector<std::string> types_of_ways;
    double interpolation_max_distance;
    n_.param<std::string>("osm_map_path",map,"");
    n_.getParam("filter_of_ways",types_of_ways);
    n_.param<double>("interpolation_max_distance", interpolation_max_distance, 1000);
    map_.setInterpolationMaxDistance(interpolation_max_distance);
    map_.setNewMap(map);
    map_.setTypeOfWays(types_of_ways);
    map_.parse();
}

void GpsCompassCorrection::setMapOrigin(ros::NodeHandle &n) {

    int origin_type;
    n.getParam("set_origin_pose", origin_type);

    if (origin_type == 3) {

        double latitude, longitude;
        n.param<double>("origin_latitude", latitude,0);
        n.param<double>("origin_longitude",longitude,0);
        map_.getCalculator()->setOrigin(latitude, longitude);
     //   map_origin_.latitude = latitude;
     //   map_origin_.longitude = longitude;
    }else{
     //   map_origin_ = map_.getNodeByID(0);
        map_.setStartPoint(0);
    }

  //  ROS_INFO("map origin type: %d [%f, %f]",origin_type, map_origin_.latitude, map_origin_.longitude);
}

void GpsCompassCorrection::tfBroadcasterCallback(const double frequence){

    const double period = 1/frequence;

    while (ros::ok()) {
            { // !!!!!! do not  remove scope (MUTEX!!!!!)
                std::lock_guard<std::mutex> lock(mutex_);
                tf_broadcaster_.sendTransform(
                        tf::StampedTransform(corrected_transform_, ros::Time::now(), parent_frame_, child_frame_));
            }
            ros::Duration(period).sleep();
        }
    }

void GpsCompassCorrection::gpsCallback(const gps_common::GPSFixPtr& gps_data){

    std::vector<geometry_msgs::Point> points;
    filter.createParticles(points);

  updateCallback(gps_data, allways_allowed_status_);
}

bool GpsCompassCorrection::forceUpdateCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    double timeout = 5;
    auto gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(gps_topic_, ros::Duration(timeout));

    if (!gps_data){
        res.success = false;
        res.message = "No gps data received after" + std::to_string(timeout) + " seconds";
        return true;
    }
    if (!updateCallback(gps_data, gps_common::GPSStatus::STATUS_FIX)){
        res.success = false;
        res.message = "No fixed gps data received";
        return true;
    }

    res.success = true;
    res.message = "Ok";
    return true;
}

bool GpsCompassCorrection::updateCallback(boost::shared_ptr<const gps_common::GPSFix> gps_data, int required_status) {

    static ros::Time time_last;
    bool was_update;
    double seconds = (ros::Time::now() - time_last).toSec();

    if ( seconds >= low_precision_period_){
        was_update = updatePosition(gps_data, min_required_status_update_);
    } else{
        was_update = updatePosition(gps_data, required_status);
    }

    if (was_update){
        ROS_DEBUG("Updated position with gps status %d after %f seconds", gps_data->status.status, seconds);
        time_last = ros::Time::now();
        return true;
    }

    return false;
}

bool GpsCompassCorrection::updatePosition(boost::shared_ptr<const gps_common::GPSFix> gps_data, int min_fix_status){

    if (!verifyGPS(gps_data, min_fix_status)){
        return false;
    }

    if (use_compass_){
        boost::shared_ptr<const sensor_msgs::Imu> imu_data = ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic_, ros::Duration(0));
        if(!imu_data){
            // init();
            ROS_ERROR("No imu data for TF correction");
            return false;
        }
        tf::Quaternion imu_quaternion;
        tf::quaternionMsgToTF(imu_data->orientation,imu_quaternion);
        sendTransform(*gps_data, &imu_quaternion);
        return true;

    } else{
        sendTransform(*gps_data);
        return true;
    }
}

bool GpsCompassCorrection::verifyGPS(boost::shared_ptr<const gps_common::GPSFix> gps_data, int min_fix_status){

    if(!gps_data){
        ROS_ERROR("No GPS data for TF correction");
        return false;
    }

    if  (gps_data->status.status < min_fix_status){
        ROS_DEBUG("Can not use %d status, Min allowed status is %d", gps_data->status.status, min_fix_status);
        return false;
    } else{
        return true;
    }
}

bool GpsCompassCorrection::setBearingCallback(osm_planner::computeBearing::Request &req, osm_planner::computeBearing::Response &res) {

    tf::Quaternion quat;
    quat.setRPY(0, 0, req.bearing);

    // If is set latitude and longitude from service msg
    if (req.latitude != 0.0 || req.longitude != 0.0){
        osm_planner::Parser::OSM_NODE node;
        node.latitude = req.latitude;
        node.longitude = req.longitude;
        sendTransform(node, &quat);
        res.message = "Transformation was updated";
        return true;

    } else {
        //Get point from GPS
        auto gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(
                gps_topic_, ros::Duration(3));
        if (!verifyGPS(gps_data, min_required_status_service_)){
            res.message = "No fixed GPS data";
            return true;
        }
        sendTransform(*gps_data, &quat);
        res.message = "Transformation is updated";
        return true;
    }
}

bool GpsCompassCorrection::computeBearingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

    static BearingCalculator bearingCalculator;

    //Get point from GPS
    auto gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(gps_topic_, ros::Duration(3));

    // Set minimal required status and verify data
    if(!verifyGPS(gps_data, min_required_status_service_)){
        res.message = "No fixed GPS data";
        res.success = false;
        return true;
    }

    // Add first point or calculate angle
    if (bearingCalculator.hasFirstPoint()) {

       tf::Quaternion quat;
       quat.setRPY(0, 0, bearingCalculator.calculate(gps_data, map_.getCalculator()));
       sendTransform(*gps_data, &quat);
       res.message = "Bearing is calculated";
       res.success = true;
       return true;

    } else {
        bearingCalculator.addPoint(gps_data);
        res.message = "Added first point, please move robot forward and call service again";
        res.success = false;
        return true;
    }
}

bool GpsCompassCorrection::autoComputeBearingCallback(std_srvs::Trigger::Request &req,
                                                     std_srvs::Trigger::Response &res) {

    BearingCalculator bearingCalculator;
    const int max_bad_status = 5;
    int count_of_reads = 0;

    auto gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(gps_topic_, ros::Duration(3));
    //Add First point
    while (!verifyGPS(gps_data, min_required_status_service_)){

        gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(gps_topic_, ros::Duration(3));

        if (count_of_reads > max_bad_status) {
            res.success = false;
            res.message = "No fixed GPS data in first point";
            return true;
        }
    }

    bearingCalculator.addPoint(gps_data);
    ROS_INFO("Added first point");

    // Move robot to next point
    robot_.move();

    //Add second point and verify data
    gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(gps_topic_, ros::Duration(3));
    count_of_reads = 0;
    while (!verifyGPS(gps_data, min_required_status_service_)){

        gps_data = ros::topic::waitForMessage<gps_common::GPSFix>(gps_topic_, ros::Duration(3));

        if (count_of_reads > max_bad_status) {
            res.success = false;
            res.message = "No fixed GPS data in second point";
            return true;
        }
    }

    // Calculate bearing and evaluate result
    double angle = bearingCalculator.calculate(gps_data, map_.getCalculator());
    //Print result
    if (angle == NAN) {
        res.message = "Computed angle: NaN";
        res.success = false;
        return true;
    } else {
        tf::Quaternion quat;
        quat.setRPY(0, 0, angle);
        sendTransform(*gps_data, &quat);
        res.message = "Computed angle: " + std::to_string(angle);
        res.success = true;
        return true;
    }
}
