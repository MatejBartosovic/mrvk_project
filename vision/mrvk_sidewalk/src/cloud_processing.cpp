//
// Created by controller on 8/3/17.
//

#include "../include/mrvk_sidewalk/cloud_processing.h"

pcl::PointXYZRGB CloudProcessing::depthToPointCloudPos(int x, int y, float depthValue) {
    pcl::PointXYZRGB point;
    point.z = (depthValue);// / (1.0f); // Convert from mm to meters
    point.x = ((x - CameraParams.cx) * point.z / CameraParams.fx);
    point.y = (((y - CameraParams.cy) * point.z / CameraParams.fy));
    return point;
}

void CloudProcessing::createVectors(int width, int height) {

    tf::StampedTransform kinect_transform;
    ROS_ERROR("Creating vectors");
    int skip = 1;
    ROS_ERROR("Creating vectors");
    for (int x = 0; x < width; x+=skip) {
        for (int y = 0; y < height; y+=skip) {
            cloud.points.push_back(depthToPointCloudPos(x, y,1));
        }
    }
    ROS_ERROR("creating vectors");

    toROSMsg (cloud, ros_cloud);
    std::string kinFrame("camera_frame");//"kinect_ir");
    std::string baseLinkFrame("base_link");
#ifdef DEBUG
    ROS_ERROR_STREAM(kinFrame);
    ROS_ERROR_STREAM(baseLinkFrame);
#endif

    ros::Time now(0);
    while (n.ok()) {
        std::string *error_code;
        //ROS_INFO_STREAM(now);
        if (this->t.waitForTransform(kinFrame, now, kinFrame, now, baseLinkFrame, ros::Duration(1))) {
            //msg->header.stamp.setNow(now);
            break;
        }
    }
    // baselink v kamerovych suradniciach
    try{
        this->t.lookupTransform(baseLinkFrame,kinFrame,
                                          ros::Time(0), kinect_transform);

    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    pcl_ros::transformPointCloud(baseLinkFrame,kinect_transform,this->ros_cloud,this->final_cloud);
    parseCloud();
}

void CloudProcessing::parseCloud() {

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(final_cloud, cloud);
    cv::Point3d point;
    //
    // ROS_ERROR_STREAM(cloud.size());
    for (int i = 0; i<cloud.size();i++){

        point.x = cloud.points[i].x-KINECT_X;
        point.y = cloud.points[i].y-KINECT_Y;
        point.z = cloud.points[i].z-KINECT_Z;

        this->vectors.push_back(point);
    }

}

pcl::PointXYZRGB CloudProcessing::returnPoint (double z,long int point) {

    double t;
    pcl::PointXYZRGB data;

    t = fabs((z-KINECT_Z)/this->vectors[point].z);
    data.z = z;
    data.x = KINECT_X+this->vectors[point].x*t;
    data.y = KINECT_Y+this->vectors[point].y*t;
	
    return data;
}

sensor_msgs::PointCloud2 CloudProcessing::getCloud() {
    return this->final_cloud;
}
