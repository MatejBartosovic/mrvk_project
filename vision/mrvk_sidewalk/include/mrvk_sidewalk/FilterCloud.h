//
// Created by root on 13.4.2019.
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"

#include <sstream>

#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"

#ifndef PROJECT_FILTERCLOUD_H
#define PROJECT_FILTERCLOUD_H


class FilterCloud {
public:
    FilterCloud();
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    ~FilterCloud();
    int trololo;
private:
    // NodeHandler
    ros::NodeHandle n_;

    //publishers
    ros::Publisher pav_map_pub_;
    ros::Publisher pav_map_cloud_pub_;

    //subscribers
    ros::Subscriber cloud_sub_;

    //methods
    void constructMap();

    //map
    nav_msgs::OccupancyGrid pav_map;
};


#endif //PROJECT_FILTERCLOUD_H
