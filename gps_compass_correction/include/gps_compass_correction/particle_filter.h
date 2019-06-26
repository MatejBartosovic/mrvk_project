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

    template<class POINTS>
    void createParticles(const tf::Transform &robot_pose, std::vector<POINTS> points){

        //if (points.size())
        visualization_msgs::Marker marker;
        static int marker_id = 0;
        marker.id = marker_id++;
        marker.action = visualization_msgs::Marker::ADD;
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

        marker.lifetime = ros::Duration(10);

        geometry_msgs::Pose robot_pose_msg;
        tf::poseTFToMsg(robot_pose, robot_pose_msg);

        //  marker_array.resize(num_of_particles_);
//        for (int i = 0; i < num_of_particles_; i++){
//
//            marker.points.push_back(generateRandom(robot_pose_msg.position.x, robot_pose_msg.position.y));
//        }

        for (auto point : points){
            for (int i = 0; i < num_of_particles_; i++){

                marker.points.push_back(generateRandom(point.x, point.y));
            }
        }

        particles_pub_.publish(marker);
    }
    geometry_msgs::Point generateRandom(double x, double y);

private:
    int num_of_particles_;
    std::vector<geometry_msgs::Point> particles_;
    visualization_msgs::MarkerArray particle_markers_;
    ros::Publisher particles_pub_;
};

#endif //PROJECT_PARTICLE_FILTER_H
