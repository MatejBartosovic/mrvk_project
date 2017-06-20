//
// Created by michal on 12.6.2017.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

ros::Publisher twist_pub;
double Kp = 1;
double targetDist = 10;
bool running = false;
bool firstMeassure = false;
double startPose;

void callback(const nav_msgs::Odometry& data) {

    geometry_msgs::Twist twist;

    double currentPose = sqrt( pow(data.pose.pose.position.x, 2.0) + pow(data.pose.pose.position.y, 2.0));

    if (!firstMeassure){
        startPose  = currentPose;
        firstMeassure = true;
    }

    double dist = currentPose - startPose;
    ROS_INFO("distance %f", dist);
    if (dist < targetDist) twist.linear.x = (targetDist - dist) * Kp;
    else ros::shutdown();

    twist_pub.publish(twist);
}

int main (int argc, char **argv){

    ros::init(argc, argv, "covariance_calculator");
    ros::NodeHandle node;

    ros::Subscriber odom_sub = node.subscribe("/mrvk/diff_drive_controller/odom", 10, callback);

    twist_pub = node.advertise<geometry_msgs::Twist>("/mrvk/diff_drive_controller/cmd_vel", 1);
    ros::spin();

}