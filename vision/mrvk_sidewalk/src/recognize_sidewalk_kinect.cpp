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

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#define EDGE_MARKER_WIDTH 6
#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

#define EDGE_PAV_VERTICAL_START 150
#define EDGE_PAV_VERTICAL_END 650
#define EDGE_MARKER_VERTICAL_POINT_DIST 50

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
    RecognizeSidewalkParams params;
    SidewalkEdges sidewalkEdges;

public:

    Sidewalk(){

        //START get parameters
        params.getParametersFromServer(n);
        //END get parameters

        //init subscribers
        sub = n.subscribe(params.image_topic, 1, &Sidewalk::kinectImageCallback,this);
        subDepth = n.subscribe(params.depth_image_topic, 1, &Sidewalk::kinectDepthImageCallback,this);

        //init publishers
        pub_img = n.advertise<sensor_msgs::Image>("video_image_topic", 1);//output image publisher
        pub_img_orig = n.advertise<sensor_msgs::Image>("video_image_orig_topic", 1);//output image publisher
        octomap_pub = n.advertise<nav_msgs::OccupancyGrid>("pavement_map", 1);//map occupancy publisher
        pub_pav_pointCloud = n.advertise<sensor_msgs::PointCloud> ("pav_pointCloud", 1);//publisher for pavement point cloud

        //point cloud header
        pointCloud_msg.header.stamp = ros::Time::now();
        pointCloud_msg.header.frame_id = "map";

    }
    ~Sidewalk(){

    }

};


void Sidewalk::sidewalkPublish()
{

    imageOrig = kinectImage.clone();

    pointCloud_msg = recognize_sidewalk_frame(&imageOrig, &imageResult, params, &sidewalkEdges);

    //publish processed image
    header.stamp = ros::Time::now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageResult);
    img_bridge.toImageMsg(img_msg);
    pub_img.publish(img_msg);//publish processed image
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageOrig);
    img_bridge.toImageMsg(img_msg_orig);
    pub_img_orig.publish(img_msg_orig);//publish original image

    //publish point cloud
    pointCloud_msg.header.stamp = ros::Time::now();
    pub_pav_pointCloud.publish(pointCloud_msg);
}

void Sidewalk::kinectImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    kinectImage = cv_bridge::toCvCopy(msg,"bgr8")->image;
    imageTime = ros::Time::now();
    sidewalkPublish();
#ifdef DEBUG

    //ROS_ERROR("Timestamp image: %f", imageTime.toSec());
#endif

}

void Sidewalk::kinectDepthImageCallback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    depthKinectImage = *depth_msg;
    depthImageTime = ros::Time::now();
    sensor_msgs::PointCloud2 cloud;
    int skip = 2;

    for (int x = 0; x < depth_msg->width; x+=skip) {
        for (int y = 0; y < depth_msg->height; y+=skip) {
            int offset = x + y * depth_msg->width;

            //calculte the x, y, z camera position based on the depth information
            sensor_msgs::PointCloud2 cloud_out;
            //cloud.data.push_back(depthToPointCloudPos(x, y, depth_msg->data[offset]));
        }
    }

#ifdef DEBUG
    ROS_ERROR_STREAM(depth_msg->data.size());
    ROS_ERROR_STREAM(depth_msg->width);
    ROS_ERROR_STREAM(depth_msg->height);
    //ROS_ERROR("Timestamp depth image: %f", depthImageTime.toSec());
#endif
}

int main(int argc, char **argv) {

    //START ros init
    ros::init(argc, argv, "recognize_sidewalk_kinect");
    Sidewalk sidewalk;
    ros::spin();

    return 0;
}




