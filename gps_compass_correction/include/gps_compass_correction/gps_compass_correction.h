//
// Created by controller on 9/7/17.
//

#ifndef PROJECT_GPS_COMPASS_CORRECTION_H
#define PROJECT_GPS_COMPASS_CORRECTION_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <osm_planner/osm_parser.h>
#include <osm_planner/computeBearing.h>
#include <std_srvs/Trigger.h>
#include <mutex>

#include "gps_compass_correction/bearing_calculator.h"
#include "gps_compass_correction/robot_commander.h"


//!  A GpcCompassCorrection class
/*!
  A Main Class for calculating correction from gps and optionally from compass
*/
class GpsCompassCorrection {
public:
    /**
     * @brief Create gps subscriber, services, tf broadcaster thread, map_origin
     * Construct RobotCommander and set all parameters from ROS param server
     */
    GpsCompassCorrection();

    /**
     * @brief Verify GPS data and calculate new transform for correction
     * @param gps_data - Some gps data
     * @param min_fix_status - Minimal required GPS status for correction
     * @return True when is transform successfully calculated
     */
    bool update(boost::shared_ptr<const gps_common::GPSFix> gps_data, int min_fix_status = gps_common::GPSStatus::STATUS_GBAS_FIX);

private:

    bool use_compass_;                     ///< It's true, then is used angle from compass for correction
    double low_precision_period_;          ///< Period of correction while is inaccessible GBAS_FIX status
    int min_required_status_update_;              ///< GPS minimal required status for correction
    int min_required_status_service_;      ///< GPS minimal required status for correction

    // Names
    std::string parent_frame_, child_frame_, target_frame_, imu_topic_, gps_topic_;

    // ROS tools
    ros::NodeHandle n_;
    ros::Subscriber gps_sub_;
    ros::ServiceServer set_bearing_service_;
    ros::ServiceServer compute_bearing_service_;
    ros::ServiceServer auto_compute_bearing_service_;

    //tf variables
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform corrected_transform_;          ///< Calculated and published transform

    //thread
    boost::shared_ptr<boost::thread> tf_broadcaster_thread_; ///< TF is broadcasted in this thread
    std::mutex mutex_;                                       ///< This mutex is used around corrected_transform

    osm_planner::Parser::OSM_NODE map_origin_;              ///< Origin - here is [0,0] world
        //distance and bearing calculator
    std::shared_ptr<osm_planner::coordinates_converters::CoordinatesConverterBase> coordinatesConverter;
    RobotCommander robot_;                                  ///< It's used in auto_compute_bearing service
                                                            ///< for auto moving with robot

    /**
     * @brief Set map origin, it's used only in constructor
     * @param n - ROS node handler. ROS param set_origin_pose decide,
     * if it will be origin in first point in map or in coordinates from param server
     */
    void setMapOrigin(ros::NodeHandle &n);

    /**
     * @brief Thread callback - periodically send TF
     * @param frequence - Rate of broadcasting
     */
    void tfBroadcasterCallback(const double frequence);

    /**
     * @brief Verify GPS msg, if it's gps status better then min_fix_status
     * Method is used in update() method
     * @param gps_data - Some gps data
     * @param min_fix_status - Minimal required GPS status for correction
     * @return True if is gps status better then min_fix_status
     */
    bool verifyGPS(boost::shared_ptr<const gps_common::GPSFix> gps_data, int min_fix_status);

    /**
     * @brief Gps subscriber callback. In callback is called method update()
     * When isn't received status GBAS_FIX during low_precision_period_,
     * then is used just status FIX for correction
     * If you mustn't use low precision, then set negative value on ROS param low_precision_period
     * @param gps_data - Some gps data from topic
     */
    void gpsCallback(const gps_common::GPSFixPtr& gps_data);

    /**
     * @brief Set bearing from service, If is obtained latitude and longitude then is use pose from service,
     * else is use first received pose from GPS
     * @param req - Latitude, Longitude and bearing
     * @param res - result message
     * @return - always True
     */
    bool setBearingCallback(osm_planner::computeBearing::Request &req, osm_planner::computeBearing::Response &res);

    /**
     * @brief Compute bearing from two GPS position
     * In first call is set first position, then is necessary move robot forward,
     * and call again
     * @param req - Empty data
     * @param res - result message
     * @return - always True
     */
    bool computeBearingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief Compute bearing automatically using RobotCommander class and publishing on topic cmd_vel
     * First get position from GPS, then robot going to move, and at the end is obtained second
     * position from GPS
     * @param req - Empty data
     * @param res - result message
     * @return - always True
     */
    bool autoComputeBearingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    /**
     * @brief A template method for calculating transform
     * @param gps_pose - Corrected gps data used for correction of pose
     * @param quat - Quaternion for correction of rotation.
     * If it is nullptr then is used original rotation
     */
    template<class N>
    void sendTransform(const N &gps_pose, const tf::Quaternion *quat = nullptr){

        tf::TransformListener listener;

        //get transformation
        tf::StampedTransform relative_transform;
        try{

            listener.waitForTransform(child_frame_, target_frame_, ros::Time(0), ros::Duration(1));
            listener.lookupTransform(child_frame_, target_frame_, ros::Time(0), relative_transform);
        }
        catch (tf::TransformException ex){
            ROS_WARN("Can't find transform between child [%s] and robot base_link frame [%s]. Exception: %s",child_frame_,target_frame_,ex.what());
            return;
        }

        //construct translation from gps
        tf::Vector3 gps_translation(coordinatesConverter->getCoordinateX(map_origin_, gps_pose),coordinatesConverter->getCoordinateY(map_origin_, gps_pose),0); //todo misov prepocet dorobit

        tf::Transform absolute_transform;
        if (quat == nullptr) {
            //get transformation
            tf::StampedTransform base_link_transform;
            try {
                listener.waitForTransform(parent_frame_, target_frame_, ros::Time(0), ros::Duration(1));
                listener.lookupTransform(parent_frame_, target_frame_, ros::Time(0), base_link_transform);
            }
            catch (tf::TransformException ex) {
                ROS_WARN("Can't find transform between parent [%s] and robot base_link frame [%s]. Exception: %s",parent_frame_, target_frame_,ex.what());
                return;
            }
            //absolute orientation
            absolute_transform = tf::Transform(base_link_transform.getRotation(), gps_translation);

        } else{
            //absolute orientation
            tf::Transform absolute_transform(*quat, gps_translation);
        }

        //compute correction
        mutex_.lock();
        corrected_transform_ = absolute_transform * relative_transform.inverse();
        mutex_.unlock();
    }

    /**
     * @brief Just debug method
     */
    void printTransform(tf::Transform &t,std::string s){

        ROS_ERROR("%s x = %lf y = %lf z = %lf x = %lf y = %lf z = %lf w = %lf",s.c_str(),t.getOrigin().x(),t.getOrigin().y(),t.getOrigin().z(),t.getRotation().x(),t.getRotation().y(),t.getRotation().z(),t.getRotation().w());

    }
};


#endif //PROJECT_GPS_COMPASS_CORRECTION_H
