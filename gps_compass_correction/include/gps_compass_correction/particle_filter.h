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


    void clear();
    void createParticles(const tf::Transform &robot_pose, int num_particles = 10, double radius = 1.0);

    template<class POINT>
    void createParticles(POINT point, int num_particles = 10, double radius = 1.0){
        for (int i = 0; i < num_particles; i++){
                particles_.push_back(generateRandom(point.x, point.y));
            }
    }

    geometry_msgs::Point generateRandom(double x, double y, double radius = 1.0);
    void publishParticles();

private:
    std::vector<geometry_msgs::Point> particles_;
    visualization_msgs::MarkerArray particle_markers_;
    ros::Publisher particles_pub_;
};

#endif //PROJECT_PARTICLE_FILTER_H
