//
// Created by controller on 6/8/19.
//


#include "gps_compass_correction/particle_filter.h"

ParticleFilter::ParticleFilter(){

    num_of_particles_ = 10;

    ros::NodeHandle n("~");
    particles_pub_ = n.advertise<visualization_msgs::Marker>("particles", 5);

}

void ParticleFilter::createParticles(std::vector<geometry_msgs::Point> points){

    //if (points.size())
    visualization_msgs::Marker marker;
    static int marker_id = 0;
    marker.id = marker_id++;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "particles";

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration(10);

    geometry_msgs::Point point_zero;
    point_zero.x = 0;
    point_zero.y = 0;

  //  marker_array.resize(num_of_particles_);
    for (int i = 0; i < num_of_particles_; i++){

        marker.points.push_back(generateRandom(point_zero));
    }

    particles_pub_.publish(marker);
}

geometry_msgs::Point ParticleFilter::generateRandom(const geometry_msgs::Point &reference_point){

    geometry_msgs::Point point;


    point.x = reference_point.x - 0.5 + ((double)rand() / RAND_MAX) * 1.0;
    point.y = reference_point.y - 0.5 + ((double)rand() / RAND_MAX) * 1.0;

    return point;
}
