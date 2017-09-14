/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <osm_planner/newTarget.h>

class Launcher{

public: Launcher(){

      ros::NodeHandle nh;
      osm_init_client = nh.serviceClient<osm_planner::newTarget>("move_base/osm/init");

    }

    bool initOSM(const sensor_msgs::NavSatFixConstPtr& fix, double bearing){

      osm_planner::newTarget initPose;
      initPose.request.longitude = fix->longitude;
      initPose.request.latitude = fix->latitude;
      initPose.request.bearing = bearing;

      return osm_init_client.call(initPose);
    }

private:
  ros::ServiceClient osm_init_client;
};

using namespace gps_common;

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;

double northing_init = 0;
double easting_init = 0;

bool initialized = false;


bool hasAngle = false;
double angle;


void initCallback(const std_msgs::Float32& msg) {

  std::string zone;
    hasAngle = true;
    angle = msg.data;
    }


void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_WARN("GPS No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if (!initialized && hasAngle) {
            northing_init = northing;
            easting_init = easting;

            Launcher launcher;
            launcher.initOSM(fix, angle);
            initialized = true;

    }

  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

   // odom.pose.pose.position.y = easting - easting_init;
//    odom.pose.pose.position.x = northing - northing_init;

    odom.pose.pose.position.y = northing - northing_init;
    odom.pose.pose.position.x = easting - easting_init;
    odom.pose.pose.position.z = 0;
    
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = 0;
    odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;

    odom_pub.publish(odom);
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

  ros::Subscriber fix_sub = node.subscribe("fix", 10, callback);

  ros::Subscriber init_sub = node.subscribe("utm/init", 10, initCallback);

  ros::spin();
}

