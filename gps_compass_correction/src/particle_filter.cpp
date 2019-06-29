//
// Created by controller on 6/8/19.
//


#include "gps_compass_correction/particle_filter.h"

#include <tf/transform_datatypes.h>

ParticleFilter::ParticleFilter(){

    ros::NodeHandle n("~");
    particles_pub_ = n.advertise<visualization_msgs::Marker>("particles", 5);

}

void ParticleFilter::clear(){
    particles_.clear();
}

void ParticleFilter::createParticles(const tf::Transform &robot_pose, int num_particles, double radius){

    geometry_msgs::Pose robot_pose_msg;
    tf::poseTFToMsg(robot_pose, robot_pose_msg);
    createParticles(robot_pose_msg.position, num_particles, radius);
}

void ParticleFilter::publishParticles(){

    visualization_msgs::Marker marker;
    marker.id = 1;
    marker.action = visualization_msgs::Marker::MODIFY;

    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "particles";

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration(0);

    marker.points = particles_;
    particles_pub_.publish(marker);
}

geometry_msgs::Point ParticleFilter::generateRandom(double x, double y, double radius){

    geometry_msgs::Point point;

    double random_radius = ((double)rand() / RAND_MAX) * radius;
    double random_angle  = -3.14 +  ((double)rand() / RAND_MAX) * 6.28;

    point.x = x + random_radius * cos(random_angle);
    point.y = y + random_radius * sin(random_angle);

    return point;
}
