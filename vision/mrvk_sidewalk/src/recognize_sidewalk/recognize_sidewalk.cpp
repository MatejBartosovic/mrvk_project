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
#include "../pavement_to_marker/pavement_to_marker.h"
#include "../pavement_to_cloud/pavement_to_cloud.h"
#include "recognize_sidewalk.h"
#include "../misc_tools/misc_tools.h"
#include "picture_segmentation.h"

//messages
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

#define EDGE_MARKER_POINT_RADIUS 2
#define EDGE_MARKER_POINT_WIDTH 4

#define DIST_GRAD -1

using namespace cv;

sensor_msgs::PointCloud recognize_sidewalk_frame(cv::Mat *imageOrig, cv::Mat *imageResultOut, RecognizeSidewalkParams params)
{

    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    pointCloud_msg.header.stamp = ros::Time::now();
    pointCloud_msg.header.frame_id = "map";

    Mat imageResult;		//Create Matrix to store processed image

    //START edge detection variables
    Point lineStart = Point(100, 100);
    Point lineEnd = Point(300, 300);
    Point lineStartRight = Point(100, 100);
    Point lineEndRight = Point(300, 300);
    int leftPoint = 0;
    int rightPoint = 0;
    int pavementCenter = 0;
    //END edge detection variables

    //frame segmentation
    //imageResult = picture_segmentation_frame(image);
    imageResult = picture_segmentation_frame_HSV(*imageOrig);

    //START draw pavement boundaries
    pavementCenter = imageResult.cols/2;

    //left
    lineStart = Point( getLeftPavementPoint(imageResult, -params.edge_start_offset + imageOrig->rows, pavementCenter, params.edge_side_offset_promile), -params.edge_start_offset + imageOrig->rows);
    lineEnd = Point( getLeftPavementPoint(imageResult, -params.edge_start_offset + imageOrig->rows - params.edge_points_dist, pavementCenter, params.edge_side_offset_promile), -params.edge_start_offset + imageOrig->rows - params.edge_points_dist);

    //right
    lineStartRight = Point( getRightPavementPoint(imageResult, -params.edge_start_offset + imageOrig->rows, pavementCenter, params.edge_side_offset_promile), -params.edge_start_offset + imageOrig->rows);
    lineEndRight = Point( getRightPavementPoint(imageResult, -params.edge_start_offset + imageOrig->rows - params.edge_points_dist, pavementCenter, params.edge_side_offset_promile), -params.edge_start_offset + imageOrig->rows - params.edge_points_dist);

    pavementCenter = getPavementCenter(lineEnd.x, lineEndRight.x, pavementCenter);

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

    int edge_cursor = 0;
    int edge_increment = 1;
    int sideOffset = (params.edge_side_offset_promile*imageResult.cols)/1000;

    while (edge_cursor < ((imageOrig->rows*params.detect_percent_of_image)/100.0))
    {
        edge_increment++;
#ifdef DEBUG
        //ROS_ERROR("recognition param %d", params.ros_parameter);
#endif
        edge_cursor = edge_cursor + pow(edge_increment*params.edge_points_dist, -0.5)*200;
        lineStart = lineEnd;
        lineEnd = Point(getLeftPavementPoint(imageResult, -params.edge_start_offset + imageOrig->rows - params.edge_points_dist - edge_cursor, pavementCenter, params.edge_side_offset_promile), -params.edge_start_offset + imageOrig->rows - params.edge_points_dist - edge_cursor);
        if (!isOpeningLeft(lineStart.x, lineEnd.x, params.sideOffest))
        {
            lineEnd.x += sideOffset;
            lineEnd.y += sideOffset;
            if (!notPavement(lineStart.x, lineEnd.x, pavementCenter, sideOffset))
            {
                line(imageResult, lineStart, lineEnd, Scalar(0, 255, 0), params.edge_marker_width, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
                line(*imageOrig, lineStart, lineEnd, Scalar(0, 255, 0), params.edge_marker_width, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
            }
        }
        cv::circle(*imageOrig, lineStart, EDGE_MARKER_POINT_RADIUS, Scalar(0, 0, 255), EDGE_MARKER_POINT_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        lineStartRight = lineEndRight;
        lineEndRight = Point(getRightPavementPoint(imageResult, -params.edge_start_offset + imageOrig->rows - params.edge_points_dist - edge_cursor, pavementCenter, params.edge_side_offset_promile), -params.edge_start_offset + imageOrig->rows - params.edge_points_dist - edge_cursor);
        if (!isOpeningRight(imageResult.cols, lineStartRight.x, lineEndRight.x, params.sideOffest))
        {
            lineEndRight.x -= sideOffset;
            lineEndRight.y += sideOffset;
            if (!notPavement(lineStart.x, lineEnd.x, pavementCenter, sideOffset))
            {
                line(imageResult, lineStartRight, lineEndRight, Scalar(0, 255, 0), params.edge_marker_width, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
                line(*imageOrig, lineStartRight, lineEndRight, Scalar(0, 255, 0), params.edge_marker_width, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
            }
        }
        cv::circle(*imageOrig, lineStartRight, EDGE_MARKER_POINT_RADIUS, Scalar(0, 0, 255), EDGE_MARKER_POINT_WIDTH, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        pavementCenter = getPavementCenter(lineEnd.x, lineEndRight.x, pavementCenter);

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

int getLeftPavementPoint(cv::Mat image, int line, int pavementCenter, int edge_side_offset_promile)
{
    for (int i = pavementCenter; i > 0; i--)
    {
        if ((image.at<cv::Vec3b>(line, i)[0] == 0))
        {
            return i;
        }
    }
    return 0;
}

int getRightPavementPoint(cv::Mat image, int line, int pavementCenter, int edge_side_offset_promile)
{
    for (int i = pavementCenter; i < image.cols; i++)
    {
        if ((image.at<cv::Vec3b>(line, i)[0] == 0))
        {
            return i;
        }
    }
    return image.cols;
}

int getPavementCenter(int leftPoint, int rightPoint, int pavementCenterLast)
{
    int pavementCenter = 0;
    if (leftPoint > rightPoint)
    {
        pavementCenter = pavementCenterLast;
    }
    else
    {
        pavementCenter = leftPoint + (rightPoint - leftPoint)/2;
        if (pavementCenter == 0)
        {
            pavementCenter = pavementCenterLast;
        }
    }
    return pavementCenter;
}

bool isOpeningLeft(int startPoint, int endPoint, int sideOffset)
{
    bool opening = false;
    if ((startPoint < sideOffset)&&(endPoint < sideOffset))
    {
        opening = true;
    }
    return opening;
}

bool isOpeningRight(int imgCols, int startPoint, int endPoint, int sideOffset)
{
    bool opening = false;
    if ((startPoint > (imgCols - sideOffset))&&(endPoint > (imgCols - sideOffset)))
    {
        opening = true;
    }
    return opening;
}

bool notPavement(int startPoint, int endPoint, int pavementCenter, int sideOffset)
{
    bool notPavement = false;
    if ((startPoint >= pavementCenter - sideOffset)&&(startPoint <= pavementCenter + sideOffset))
    {
        if ((endPoint >= pavementCenter - sideOffset)&&(endPoint <= pavementCenter + sideOffset))
        {
            notPavement = true;
        }
    }
    return notPavement;
}