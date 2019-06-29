//
// Created by controller on 6/8/19.
//


#include "gps_compass_correction/particle_filter.h"

#include <tf/transform_datatypes.h>

namespace particle_filter{
    void particlesToMsg(const Particles &particles, std::vector<geometry_msgs::Point> &points, double weight){
        points.clear();
        for (auto particle : particles) {
            if (particle.weight >= weight)
                points.push_back(particle.point);
        }
    }
}

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

    particle_filter::particlesToMsg(particles_, marker.points, 0.0);
    particles_pub_.publish(marker);
}

void ParticleFilter::filter(const geometry_msgs::Point &sensor_data){

    // Calculate distances
    double max_distance = 0;
    for (auto &particle : particles_){
        particle.distance = computeDistance(particle.point, sensor_data);
        if (particle.distance > max_distance){
            max_distance = particle.distance;
        }
    }

    // Calculate weights
    for (auto &particle : particles_){
        particle.weight = 1 - particle.distance / max_distance;
    }
}

particle_filter::Particle ParticleFilter::generateRandom(double x, double y, double radius){

    particle_filter::Particle particle;

    double random_radius = ((double)rand() / RAND_MAX) * radius;
    double random_angle  = -3.14 +  ((double)rand() / RAND_MAX) * 6.28;

    particle.point.x = x + random_radius * cos(random_angle);
    particle.point.y = y + random_radius * sin(random_angle);
    particle.distance = 0;
    particle.weight = 0;

    return particle;
}

double ParticleFilter::computeDistance(geometry_msgs::Point point1, geometry_msgs::Point point2){
    return sqrt(pow(point1.x - point2.x, 2.0) + pow(point1.y - point2.y, 2.0));
}
