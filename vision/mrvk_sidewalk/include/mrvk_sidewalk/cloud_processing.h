//
// Created by controller on 8/3/17.
//

#ifndef PROJECT_CLOUD_PROCESSING_H
#define PROJECT_CLOUD_PROCESSING_H

//pointcloud - pavement
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include "tf/message_filter.h"
#include <cv_bridge/cv_bridge.h>


#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"




#define KINECT_X 0.1
#define KINECT_Y -0.07
#define KINECT_Z 0.663

class CloudProcessing{
public:

    CloudProcessing():t(){

    }
    ~CloudProcessing(){

    }
    pcl::PointXYZRGB depthToPointCloudPos(int x, int y, float depthValue);
    void createVectors(int width, int height);
    sensor_msgs::PointCloud2 getCloud();
    pcl::PointXYZRGB returnPoint (double z, long int point);


private:

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::PointCloud2 ros_cloud, final_cloud;
    ros::NodeHandle n;


    std::vector< cv::Point3d > vectors;


    tf::TransformListener t;
    const struct CameraParams {
        float cx = 940.9387918663755;
        float cy = 533.8446166443484f;
        float fx = 1046.881763495155f;
        float fy = 1046.524330950266;
        float k1 = 0.0905474;
        float k2 = -0.26819;
        float k3 = 0.0950862;
        float p1 = 0.0;
        float p2 = 0.0;
    }CameraParams;


    void parseCloud();
};

#endif //PROJECT_CLOUD_PROCESSING_H
