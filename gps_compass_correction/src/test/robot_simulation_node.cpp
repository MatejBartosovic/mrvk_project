//
// Created by controller on 5/1/19.
//

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <gps_common/GPSFix.h>



double randGen(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_simulator");
    ros::NodeHandle n;

    // GSP publisher
    ros::Publisher gps_pub = n.advertise<gps_common::GPSFix>("/gps", 1);
    gps_common::GPSFix gps_msg;
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = "world";
    gps_msg.header.seq = 0;
    gps_msg.latitude = 48.1531010533;
    gps_msg.longitude = 17.0743728117;
    n.getParam("/gps_compass_correction_node/origin_latitude", gps_msg.latitude);
    n.getParam("/gps_compass_correction_node/origin_longitude", gps_msg.longitude);
    //gps_msg.status.status = gps_common::GPSStatus::STATUS_GBAS_FIX;
    gps_msg.status.status = gps_common::GPSStatus::STATUS_FIX;
    // Robot pose broadcaster
    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(20);

    double x = 0;
    double y = 0;
    double theta = 0;
    double u = 0;
    double counter = 0;

    while(ros::ok()){

        transform.setOrigin( tf::Vector3( x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0,0, theta);
        transform.setRotation(q);
      //  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/map"));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/base_link"));

        if (counter++ > 10) {
            u = randGen(0, 0.1);
            theta += randGen(-M_PI / 8, M_PI / 8);
            x += u * cos(theta);
            y += u * sin(theta);
            counter = 0;

            gps_msg.header.stamp = ros::Time::now();
            gps_msg.header.seq++;
            gps_pub.publish(gps_msg);
        }
        rate.sleep();
    }

    return 0;
}