//
// Created by controller on 6/8/19.
//

#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

namespace particle_filter{
    struct Particle{
        geometry_msgs::Point point;
        double distance;
        double weight;
    };
    typedef std::vector<Particle> Particles;
    void particlesToMsg(const Particles &particles, std::vector<geometry_msgs::Point> &points, double weight);

}

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

    void filter(const geometry_msgs::Point &sensor_data);

    void publishParticles();
    void publishNewParticles();
    void publishParticles(const particle_filter::Particles &particles, const std_msgs::ColorRGBA &color, int id);


private:
    particle_filter::Particles particles_;
    particle_filter::Particles new_particles_;

    ros::Publisher particles_pub_;

    const int NUM_OF_NEW_PARTICLES = 30;

    double getRandom(double offset, double dispersion);
    particle_filter::Particle generateRandom(double x, double y, double radius = 1.0);
    double computeDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);
};

#endif //PROJECT_PARTICLE_FILTER_H
