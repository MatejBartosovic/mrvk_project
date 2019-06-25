//
// Created by controller on 6/8/19.
//

#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>


class ParticleFilter{
public:
    ParticleFilter();
    void createParticles(std::vector<geometry_msgs::Point> points);
    geometry_msgs::Point generateRandom(const geometry_msgs::Point &reference_point);

private:
    int num_of_particles_;
    std::vector<geometry_msgs::Point> particles_;
    visualization_msgs::MarkerArray particle_markers_;
    ros::Publisher particles_pub_;
};

#endif //PROJECT_PARTICLE_FILTER_H
