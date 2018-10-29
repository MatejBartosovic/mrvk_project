//
// Created by controller on 10/28/18.
//

#include "gps_compass_correction/robot_commander.h"
#include <geometry_msgs/Twist.h>

RobotCommander::RobotCommander(ros::NodeHandle &n){

    n.param<double>("moving_time", moving_time_, 2);
    n.param<double>("moving_velocity", velocity_, 1);

    std::string cmd_vel_topic_name;
    n.param<std::string>("cmd_vel_topic", cmd_vel_topic_name, "cmd_vel");

    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1);
}

void RobotCommander::move(){

    //Set twist message
    geometry_msgs::Twist twist;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    twist.linear.z = 0;
    twist.linear.y = 0;
    twist.linear.x = velocity_;

    //set timing and publish cmd_vel
    ros::Time start = ros::Time::now();
    ros::Time current = ros::Time::now();
    ros::Rate rate(PUBLISHER_RATE_);

    while ((current - start).toSec() < moving_time_){

        cmd_vel_pub_.publish(twist);
        rate.sleep();
        current = ros::Time::now();
    }

    //stop publishing cmd_vel
    twist.linear.x = 0;
    cmd_vel_pub_.publish(twist);
}