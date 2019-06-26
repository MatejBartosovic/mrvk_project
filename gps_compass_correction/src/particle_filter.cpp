//
// Created by controller on 6/8/19.
//


#include "gps_compass_correction/particle_filter.h"

#include <tf/transform_datatypes.h>

ParticleFilter::ParticleFilter(){

    num_of_particles_ = 10;

    ros::NodeHandle n("~");
    particles_pub_ = n.advertise<visualization_msgs::Marker>("particles", 5);

}

geometry_msgs::Point ParticleFilter::generateRandom(double x, double y){

    geometry_msgs::Point point;

    point.x = x - 0.5 + ((double)rand() / RAND_MAX) * 1.0;
    point.y = y - 0.5 + ((double)rand() / RAND_MAX) * 1.0;

    return point;
}
