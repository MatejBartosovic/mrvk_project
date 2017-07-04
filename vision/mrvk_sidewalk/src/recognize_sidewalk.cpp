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
#include "recognize_sidewalk.h"
#include "misc_tools/misc_tools.h"

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#define EDGE_MARKER_WIDTH 6
#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

#define EDGE_PAV_VERTICAL_START 150
#define EDGE_PAV_VERTICAL_END 650
#define EDGE_MARKER_VERTICAL_POINT_DIST 50

#define EDGE_START_OFFSET 30
#define EDGE_END 300

#define DIST_GRAD -1

using namespace cv;

sensor_msgs::PointCloud recognize_sidewalk_frame(cv::Mat image, cv::Mat *imageResultOut)
{

    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    pointCloud_msg.header.stamp = ros::Time::now();
    pointCloud_msg.header.frame_id = "map";

    Mat imageResult;		//Create Matrix to store processed image
    Mat imageOrig;			//Create Matrix to store orig image with detected pavement

    //START edge detection variables
    Point lineStart = Point(100, 100);
    Point lineEnd = Point(300, 300);
    Point lineStartRight = Point(100, 100);
    Point lineEndRight = Point(300, 300);
    int num_of_edge_points = 10;//(EDGE_PAV_VERTICAL_END - EDGE_PAV_VERTICAL_START - EDGE_MARKER_VERTICAL_POINT_DIST)/EDGE_MARKER_VERTICAL_POINT_DIST;
    int leftPoint = 0;
    int rightPoint = 0;
    //END edge detection variables

    imageOrig = image.clone();

    cv::blur(image,imageResult,cv::Size(60, 60));//blurr image
    //remove red and blue
    for(int i = 0; i < imageResult.size().width; i++)
    {
        for(int j = 0; j < imageResult.size().height; j++)
        {
            //imageResult.at<cv::Vec3b>(j,i)[0]= 0;
            imageResult.at<cv::Vec3b>(j,i)[1]= 0;
            imageResult.at<cv::Vec3b>(j,i)[2]= 0;
            //increase contrast
            if (imageResult.at<cv::Vec3b>(j,i)[0] > 120)
            {
                imageResult.at<cv::Vec3b>(j,i)[0] = 255;
            }
            else
            {
                imageResult.at<cv::Vec3b>(j,i)[0] = 0;
            }
        }
    }

    //START draw pavement boundaries
    //left
    lineStart = Point( getLeftPavementPoint(imageResult, EDGE_START_OFFSET + image.rows), EDGE_START_OFFSET + image.rows);
    lineEnd = Point( getLeftPavementPoint(imageResult, EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST), EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST);
    line(imageResult, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
    line(imageOrig, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

    //right
    lineStartRight = Point( getRightPavementPoint(imageResult, EDGE_START_OFFSET + image.rows), EDGE_START_OFFSET + image.rows);
    lineEndRight = Point( getRightPavementPoint(imageResult, EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST), EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST);
    line(imageResult, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
    line(imageOrig, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

    //putPavementFragmentIntoCloud
    ///vypocet prvych pozicii x
    pavFragment pavFragmentC;
    pavFragmentC.cm.left.start.x = -1000;
    pavFragmentC.cm.right.start.x = 1000;
    pavFragmentC.cm.left.start.y = 50000;
    pavFragmentC.cm.right.start.y = 50000;
    pointCloud_msg.points.clear();
    pavFragmentC.pix.left.start = lineStart;
    pavFragmentC.pix.left.end = lineEnd;
    pavFragmentC.pix.right.start = lineStartRight;
    pavFragmentC.pix.right.end = lineEndRight;
    //changeToCmX(&pavFragmentC);
    pavFragmentC.cm.left.start.x = pavFragmentC.cm.left.end.x;
    pavFragmentC.cm.right.start.x = pavFragmentC.cm.right.end.x;
    //putPavementFragmentIntoCloud(&pointCloud_msg, &pavFragmentC);

    //for (int i = 0; i < num_of_edge_points; i++)
    int edge_cursor = 0;
    int edge_increment = 0;
    while (edge_cursor > EDGE_END)
    {
        edge_cursor = edge_cursor + pow(edge_increment*10.0, -0.5)*200;
        lineStart = lineEnd;
        lineEnd = Point(getLeftPavementPoint(imageResult, EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST - edge_cursor), EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST - edge_cursor);
        line(imageResult, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        line(imageOrig, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

        lineStartRight = lineEndRight;
        lineEndRight = Point(getRightPavementPoint(imageResult, EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST - edge_cursor), EDGE_START_OFFSET + image.rows - EDGE_MARKER_VERTICAL_POINT_DIST - edge_cursor);
        line(imageResult, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        line(imageOrig, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

        //put pavement fragment into cloud
        pavFragmentC.pix.left.start = lineStart;
        pavFragmentC.pix.left.end = lineEnd;
        pavFragmentC.pix.right.start = lineStartRight;
        pavFragmentC.pix.right.end = lineEndRight;
        //putPavementFragmentIntoCloud(&pointCloud_msg, &pavFragmentC);
    }

    *imageResultOut = imageResult;

    return pointCloud_msg;
}

int getLeftPavementPoint(cv::Mat image, int line)
{
    int isGrass = 0;
    int isEdge = 0;
    for (int i = 1; i < image.size().width; i++)
    {
        if (image.at<cv::Vec3b>(line, i)[0] == 0)
        {
            isGrass = 1;
        }
        else if ((image.at<cv::Vec3b>(line, i)[0] == 255) && (isGrass == 1) && (isEdge == 0))
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
        if (image.at<cv::Vec3b>(line, i)[0] == 0)
        {
            isGrass = 1;
        }
        else if ((image.at<cv::Vec3b>(line, i)[0] == 255) && (isGrass == 1) && (isEdge == 0))
        {
            isGrass = 0;
            isEdge = 1;
            return i;
        }
    }
    return 0;
}




/*sensor_msgs::PointCloud recognize_sidewalk_frame(cv::Mat image, cv::Mat *imageResultOut)
{

    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    pointCloud_msg.header.stamp = ros::Time::now();
    pointCloud_msg.header.frame_id = "map";

    Mat imageResult;		//Create Matrix to store processed image
    Mat imageOrig;			//Create Matrix to store orig image with detected pavement

    //START edge detection variables
    Point lineStart = Point(100, 100);
    Point lineEnd = Point(300, 300);
    Point lineStartRight = Point(100, 100);
    Point lineEndRight = Point(300, 300);
    int num_of_edge_points = (EDGE_PAV_VERTICAL_END - EDGE_PAV_VERTICAL_START - EDGE_MARKER_VERTICAL_POINT_DIST)/EDGE_MARKER_VERTICAL_POINT_DIST;
    int leftPoint = 0;
    int rightPoint = 0;
    //END edge detection variables

    imageOrig = image.clone();

    cv::blur(image,imageResult,cv::Size(60, 60));//blurr image
    //remove red and blue
    for(int i = 0; i < imageResult.size().width; i++)
    {
        for(int j = 0; j < imageResult.size().height; j++)
        {
            //imageResult.at<cv::Vec3b>(j,i)[0]= 0;
            imageResult.at<cv::Vec3b>(j,i)[1]= 0;
            imageResult.at<cv::Vec3b>(j,i)[2]= 0;
            //increase contrast
            if (imageResult.at<cv::Vec3b>(j,i)[0] > 120)
            {
                imageResult.at<cv::Vec3b>(j,i)[0] = 255;
            }
            else
            {
                imageResult.at<cv::Vec3b>(j,i)[0] = 0;
            }
        }
    }

    //START draw pavement boundaries
    //left
    lineStart = Point( getLeftPavementPoint(imageResult, EDGE_PAV_VERTICAL_START), EDGE_PAV_VERTICAL_START);
    lineEnd = Point( getLeftPavementPoint(imageResult, EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST), EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST);
    line(imageResult, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
    line(imageOrig, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

    //right
    lineStartRight = Point( getRightPavementPoint(imageResult, EDGE_PAV_VERTICAL_START), EDGE_PAV_VERTICAL_START);
    lineEndRight = Point( getRightPavementPoint(imageResult, EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST), EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST);
    line(imageResult, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
    line(imageOrig, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

    //putPavementFragmentIntoCloud
    ///vypocet prvych pozicii x
    pavFragment pavFragmentC;
    pavFragmentC.cm.left.start.x = -1000;
    pavFragmentC.cm.right.start.x = 1000;
    pavFragmentC.cm.left.start.y = 50000;
    pavFragmentC.cm.right.start.y = 50000;
    pointCloud_msg.points.clear();
    pavFragmentC.pix.left.start = lineStart;
    pavFragmentC.pix.left.end = lineEnd;
    pavFragmentC.pix.right.start = lineStartRight;
    pavFragmentC.pix.right.end = lineEndRight;
    changeToCmX(&pavFragmentC);
    pavFragmentC.cm.left.start.x = pavFragmentC.cm.left.end.x;
    pavFragmentC.cm.right.start.x = pavFragmentC.cm.right.end.x;
    putPavementFragmentIntoCloud(&pointCloud_msg, &pavFragmentC);

    for (int i = 0; i < num_of_edge_points; i++)
    {
        lineStart = lineEnd;
        lineEnd = Point(getLeftPavementPoint(imageResult, EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST + EDGE_MARKER_VERTICAL_POINT_DIST*i), EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST + EDGE_MARKER_VERTICAL_POINT_DIST*i);
        line(imageResult, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        line(imageOrig, lineStart, lineEnd, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

        lineStartRight = lineEndRight;
        lineEndRight = Point(getRightPavementPoint(imageResult, EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST + EDGE_MARKER_VERTICAL_POINT_DIST*i), EDGE_PAV_VERTICAL_START + EDGE_MARKER_VERTICAL_POINT_DIST + EDGE_MARKER_VERTICAL_POINT_DIST*i);
        line(imageResult, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        line(imageOrig, lineStartRight, lineEndRight, Scalar(0, 255, 0), EDGE_MARKER_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);

        //put pavement fragment into cloud
        pavFragmentC.pix.left.start = lineStart;
        pavFragmentC.pix.left.end = lineEnd;
        pavFragmentC.pix.right.start = lineStartRight;
        pavFragmentC.pix.right.end = lineEndRight;
        putPavementFragmentIntoCloud(&pointCloud_msg, &pavFragmentC);
    }

    *imageResultOut = imageResult;

    return pointCloud_msg;
}*/