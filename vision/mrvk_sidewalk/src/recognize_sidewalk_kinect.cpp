//ros
#include <ros/ros.h>

//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

//octomap - pavement
#include <nav_msgs/OccupancyGrid.h>

//pointcloud - pavement
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>

//looking for home directory
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>

//file libraries
#include <iostream>
#include <fstream>

//local libraries
#include "pavement_to_marker/pavement_to_marker.h"
#include "pavement_to_cloud/pavement_to_cloud.h"
#include "misc_tools/misc_tools.h"
#include "recognize_sidewalk/recognize_sidewalk.h"
#include "RecognizeSidewalkParams.h"
#include "recognize_sidewalk/SidewalkEdge.h"
#include "../include/mrvk_sidewalk/cloud_processing.h"

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#define EDGE_MARKER_WIDTH 6
#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

#define EDGE_PAV_VERTICAL_START 150
#define EDGE_PAV_VERTICAL_END 250 //for kinect 650
#define EDGE_MARKER_VERTICAL_POINT_DIST 50

#define PIC_HEIGHT_PIX2 320 //for kinect 960
#define PIC_HALF_WIDTH_PIX2 240 //for kinect 540

using namespace cv;

//class CvImage
//{
//    sensor_msgs::ImagePtr toImageMsg() const;
//
//    // Overload mainly intended for aggregate messages that contain
//    // a sensor_msgs::Image as a member.
//    void toImageMsg(sensor_msgs::Image& ros_image) const;
//};

class Sidewalk
{
private:


    // methods
    void kinectImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void kinectDepthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg);
    void sidewalkPublish();
    short valid_data;

    ros::NodeHandle n;

    ros::Publisher pub_img;//output image publisher
    ros::Publisher pub_img_orig;//output image publisher
    ros::Publisher octomap_pub;//map occupancy publisher
    ros::Publisher pub_pav_pointCloud;//publisher for pavement point cloud

    //subscribers
    ros::Subscriber sub;
    ros::Subscriber subDepth;
    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    sensor_msgs::Image img_msg_orig;
    std_msgs::Header header;

    Mat image;          //Create Matrix to store image
    Mat imageResult;		//Create Matrix to store processed image
    Mat imageOrig;			//Create Matrix to store orig image with detected pavement

    cv::Mat kinectImage;
    sensor_msgs::Image depthKinectImage;
    ros::Time imageTime;
    ros::Time depthImageTime;
//ROS_ERROR("Sidewalk init params");
    RecognizeSidewalkParams params;
    SidewalkEdges sidewalkEdges;
    CloudProcessing cloudProcessing;


public:

    Sidewalk():cloudProcessing(){
        //ROS_ERROR_STREAM("SIDEWALKINIT");
        //START get parameters
ROS_ERROR("Sidewalk init");
        params.getParametersFromServer(n);
        valid_data = 0;
        //END get parameters

        //init subscribers
//char autobusChar[30];
//autobusChar = params.image_topic.c_str();
ROS_ERROR("Sidewalk topic %s", params.image_topic.c_str());
ROS_INFO("Sidewalk topic %s", params.image_topic.c_str());
        sub = n.subscribe(params.image_topic.c_str(), 1, &Sidewalk::kinectImageCallback,this);
//        subDepth = n.subscribe(params.depth_image_topic, 1, &Sidewalk::kinectDepthImageCallback,this);

        //init publishers

        pub_img = n.advertise<sensor_msgs::Image>("video_image_topic", 1);//output image publisher
        pub_img_orig = n.advertise<sensor_msgs::Image>("video_image_orig_topic", 1);//output image publisher
 //       octomap_pub = n.advertise<nav_msgs::OccupancyGrid>("pavement_map", 1);//map occupancy publisher
        pub_pav_pointCloud = n.advertise<sensor_msgs::PointCloud2> ("pav_pointCloud", 1);//publisher for pavement point cloud
        //point cloud header
        pointCloud_msg.header.stamp = ros::Time::now();
        pointCloud_msg.header.frame_id = "map";
ROS_ERROR("Sidewalk init7");
        cloudProcessing.createVectors(640,480);//(1920,1080);
ROS_ERROR("Sidewalk init8");
    }
    ~Sidewalk(){

    }

};


void Sidewalk::sidewalkPublish()
{

    imageOrig = kinectImage.clone();

    valid_data = recognize_sidewalk_frame(&imageOrig, &imageResult, &params, &sidewalkEdges, n);

    //publish processed image
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageResult);
    img_bridge.toImageMsg(img_msg);
    pub_img.publish(img_msg);//publish processed image
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageOrig);
    img_bridge.toImageMsg(img_msg_orig);
    pub_img_orig.publish(img_msg_orig);//publish original image

}

void Sidewalk::kinectImageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    kinectImage = cv_bridge::toCvCopy(msg,"bgr8")->image;
    imageTime = ros::Time::now();
    sidewalkPublish();

    sensor_msgs::PointCloud2 final_cloud = cloudProcessing.getCloud();
    sensor_msgs::PointCloud2 final_cloud2;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;

    long int data_point;
    if(valid_data == 0) {
        for (int i = 0; i < sidewalkEdges.left.validPoints.size(); i++) {
            data_point =
                    (PIC_HEIGHT_PIX2 - sidewalkEdges.left.validPoints[i].x) * PIC_HALF_WIDTH_PIX2 + (PIC_HALF_WIDTH_PIX2 - sidewalkEdges.left.validPoints[i].y);
            cloud.push_back(cloudProcessing.returnPoint(0, data_point));

        }

        for (int i = 0; i < sidewalkEdges.right.validPoints.size(); i++) {
            data_point =
                    (PIC_HEIGHT_PIX2 - sidewalkEdges.right.validPoints[i].x) * PIC_HALF_WIDTH_PIX2 + (PIC_HALF_WIDTH_PIX2 - sidewalkEdges.right.validPoints[i].y);
            cloud.push_back(cloudProcessing.returnPoint(0, data_point));

        }
    }


    toROSMsg (cloud, final_cloud2);
    final_cloud2.header.frame_id = "base_stabilized";
    final_cloud2.header.stamp = ros::Time::now();
    pub_pav_pointCloud.publish(final_cloud2);

#ifdef DEBUG

    //ROS_ERROR("Timestamp image: %f", imageTime.toSec());
#endif

}

void Sidewalk::kinectDepthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{

#ifdef DEBUG
    //ROS_ERROR_STREAM(depth_msg->data.size());
    //ROS_ERROR_STREAM(depth_msg->width);
    //ROS_ERROR_STREAM(depth_msg->height);
    //ROS_ERROR("Timestamp depth image: %f", depthImageTime.toSec());
#endif
}

int main(int argc, char **argv) {

    //START ros init
    ros::init(argc, argv, "recognize_sidewalk_kinect");
    ROS_ERROR("Initialized node recognize_sidewalk_kinect.");
    Sidewalk sidewalk;
    ROS_ERROR("Initialized sidewalk class.");
    ros::spin();

    return 0;
}


//Errors

//terminate called after throwing an instance of 'ros::InvalidNameException'
//  what():  Character [#] at element [24] is not valid in Graph Resource Name [/kinect2/qhd/image_color#/usb_cam/image_raw#].  Valid characters are a-z, A-Z, 0-9, / and _.
