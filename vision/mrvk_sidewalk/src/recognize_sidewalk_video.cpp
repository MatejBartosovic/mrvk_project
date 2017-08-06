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

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#define EDGE_MARKER_WIDTH 6
#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

#define EDGE_PAV_VERTICAL_START 150
#define EDGE_PAV_VERTICAL_END 650
#define EDGE_MARKER_VERTICAL_POINT_DIST 50

using namespace cv;

class CvImage
{
    sensor_msgs::ImagePtr toImageMsg() const;

    void toImageMsg(sensor_msgs::Image& ros_image) const;
};

//int getLeftPavementPoint(cv::Mat image, int line);
//int getRightPavementPoint(cv::Mat image, int line);

int main(int argc, char **argv) {

    //START ros init
    ros::init(argc, argv, "recognize_sidewalk_video");
    ros::NodeHandle n;

    //START get parameters
    RecognizeSidewalkParams params;
    params.getParametersFromServer(n);
    ros::Rate loop_rate(params.spinFreq);
    //END get parameters

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    sensor_msgs::Image img_msg_orig;
    std_msgs::Header header;

    //init publishers
    ros::Publisher pub_img = n.advertise<sensor_msgs::Image>("video_image_topic", 1);//output image publisher
    ros::Publisher pub_img_orig = n.advertise<sensor_msgs::Image>("video_image_orig_topic", 1);//output image publisher
    ros::Publisher octomap_pub = n.advertise<nav_msgs::OccupancyGrid>("pavement_map", 1);//map occupancy publisher
    ros::Publisher pub_pav_pointCloud = n.advertise<sensor_msgs::PointCloud> ("pav_pointCloud", 1);//publisher for pavement point cloud
    //END ros init

    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    pointCloud_msg.header.stamp = ros::Time::now();
    pointCloud_msg.header.frame_id = "map";

    //START video directory
    std::string videoDirectory;
    videoDirectory = get_directory("/Videos/MRVKroute/", "obed", "", "mp4");
    //END video directory

    Mat image;          //Create Matrix to store image
    Mat imageResult;		//Create Matrix to store processed image
    Mat imageOrig;			//Create Matrix to store orig image with detected pavement

    VideoCapture cap;          //initialize capture
    cap.open(videoDirectory.c_str());

    //START edge detection variables
    SidewalkEdges sidewalkEdges;
    Point lineStart;// = Point(100, 100);
    Point lineEnd;// = Point(300, 300);
    Point lineStartRight;// = Point(100, 100);
    Point lineEndRight;// = Point(300, 300);
    int num_of_edge_points = (EDGE_PAV_VERTICAL_END - EDGE_PAV_VERTICAL_START - EDGE_MARKER_VERTICAL_POINT_DIST)/EDGE_MARKER_VERTICAL_POINT_DIST;
    int leftPoint = 0;
    int rightPoint = 0;

    //END edge detection variables

    while (ros::ok())
    {
        cap >> image;
        if (image.empty())
        {
            //end of video
            break;
        }
        imageOrig = image.clone();

        pointCloud_msg = recognize_sidewalk_frame(&imageOrig, &imageResult, params, &sidewalkEdges);

        //publish processed image
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageResult);
        img_bridge.toImageMsg(img_msg);
        pub_img.publish(img_msg);//publish processed image
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageOrig);
        img_bridge.toImageMsg(img_msg_orig);
        pub_img_orig.publish(img_msg_orig);//publish original image
        //usleep(10000);

        //publish point cloud
        pointCloud_msg.header.stamp = ros::Time::now();
        pub_pav_pointCloud.publish(pointCloud_msg);

        //check ros events
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}

/*int getLeftPavementPoint(cv::Mat image, int line)
{
    int isGrass = 0;
    int isEdge = 0;
    for (int i = 0; i < image.size().width; i++)
    {
        if (image.at<cv::Vec3b>(line,i)[0] == 0)
        {
            isGrass = 1;
        }
        else if ((image.at<cv::Vec3b>(line,i)[0] == 255) && (isGrass == 1) && (isEdge == 0))
        {
            isGrass = 0;
            isEdge = 1;
            return i;
        }
    }
    return 0;
}

int getRightPavementPoint(cv::Mat image, int line)
{
    int isGrass = 0;
    int isEdge = 0;
    for (int i = image.size().width; i > 0; i--)
    {
        if (image.at<cv::Vec3b>(line,i)[0] == 0)
        {
            isGrass = 1;
        }
        else if ((image.at<cv::Vec3b>(line,i)[0] == 255) && (isGrass == 1) && (isEdge == 0))
        {
            isGrass = 0;
            isEdge = 1;
            return i;
        }
    }
    return 0;
}*/


