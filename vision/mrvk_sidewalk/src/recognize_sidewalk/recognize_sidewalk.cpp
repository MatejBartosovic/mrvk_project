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

#include <vector>
#include <queue>

//local libraries
#include "../pavement_to_marker/pavement_to_marker.h"
#include "../pavement_to_cloud/pavement_to_cloud.h"
#include "recognize_sidewalk.h"
#include "../misc_tools/misc_tools.h"
#include "picture_segmentation.h"

//messages
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "../misc_tools/misc_tools.h"

#define EDGE_MARKER_POINT_RADIUS 2
#define EDGE_MARKER_POINT_WIDTH 4

#define DIST_GRAD -1

using namespace cv;

short recognize_sidewalk_frame(cv::Mat *imageOrig, cv::Mat *imageResultOut, RecognizeSidewalkParams *params, SidewalkEdges *sidewalkEdges)
{
    //std::string imFile = get_directory("/Pictures/", "calibration", "", "jpg");
    //imwrite( imFile.c_str(), *imageOrig);
    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    pointCloud_msg.header.stamp = ros::Time::now();
    pointCloud_msg.header.frame_id = "map";
    std::queue<std::vector<LineStructure> > leftEdge;
    std::queue<std::vector<LineStructure> > rightEdge;

    Mat imageResult;		//Create Matrix to store processed image

    //START edge detection variables
    Point lineStart = Point(100, 100);
    Point lineEnd = Point(300, 300);
    Point lineStartRight = Point(100, 100);
    Point lineEndRight = Point(300, 300);
    int leftPoint = 0;
    int rightPoint = 0;
    int pavementCenter = 0;
    sidewalkEdges->left.clearEdge();
    sidewalkEdges->right.clearEdge();
    //END edge detection variables
	
	short isValid = 0; // 0 valid OK, -1 invalid
    //frame segmentation
    //imageResult = picture_segmentation_frame(image);
    //imageResult = picture_segmentation_frame_HSV(*imageOrig);
    //imageResult = picture_segmentation_frame_c1c2c3(*imageOrig);
	imageResult = picture_segmentation_frame_c1c2c3_check(*imageOrig, &isValid, sidewalkEdges);

    //START draw pavement boundaries
    pavementCenter = imageResult.cols/2;

    //left
    lineStart = Point( getLeftPavementPoint(imageResult, -params->edge_start_offset + imageOrig->rows, pavementCenter, params->edge_side_offset_promile), -params->edge_start_offset + imageOrig->rows);
    lineEnd = Point( getLeftPavementPoint(imageResult, -params->edge_start_offset + imageOrig->rows - params->edge_points_dist, pavementCenter, params->edge_side_offset_promile), -params->edge_start_offset + imageOrig->rows - params->edge_points_dist);

    //right
    lineStartRight = Point( getRightPavementPoint(imageResult, -params->edge_start_offset + imageOrig->rows, pavementCenter, params->edge_side_offset_promile), -params->edge_start_offset + imageOrig->rows);
    lineEndRight = Point( getRightPavementPoint(imageResult, -params->edge_start_offset + imageOrig->rows - params->edge_points_dist, pavementCenter, params->edge_side_offset_promile), -params->edge_start_offset + imageOrig->rows - params->edge_points_dist);

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
    int sideOffset = (params->edge_side_offset_promile*imageResult.cols)/1000;

    while (edge_cursor < ((imageOrig->rows*params->detect_percent_of_image)/100.0))
    {
        edge_increment++;
#ifdef DEBUG
        //ROS_ERROR("recognition param %d", params->ros_parameter);
#endif
        edge_cursor = edge_cursor + pow(edge_increment*params->edge_points_dist, -0.5)*200;
        lineStart = lineEnd;
        lineEnd = Point(getLeftPavementPoint(imageResult, -params->edge_start_offset + imageOrig->rows - params->edge_points_dist - edge_cursor, pavementCenter, params->edge_side_offset_promile), -params->edge_start_offset + imageOrig->rows - params->edge_points_dist - edge_cursor);
        sidewalkEdges->left.new_line();
        sidewalkEdges->left.getEdgeRaw()->at(sidewalkEdges->left.getEdgeRaw()->size() - 1).start = lineStart;
        sidewalkEdges->left.getEdgeRaw()->at(sidewalkEdges->left.getEdgeRaw()->size() - 1).end = lineEnd;
        lineStartRight = lineEndRight;
        lineEndRight = Point(getRightPavementPoint(imageResult, -params->edge_start_offset + imageOrig->rows - params->edge_points_dist - edge_cursor, pavementCenter, params->edge_side_offset_promile), -params->edge_start_offset + imageOrig->rows - params->edge_points_dist - edge_cursor);
        sidewalkEdges->right.new_line();
        sidewalkEdges->right.getEdgeRaw()->at(sidewalkEdges->right.getEdgeRaw()->size() - 1).start = lineStartRight;
        sidewalkEdges->right.getEdgeRaw()->at(sidewalkEdges->right.getEdgeRaw()->size() - 1).end = lineEndRight;
        pavementCenter = getPavementCenter(lineEnd.x, lineEndRight.x, pavementCenter);

        //put pavement fragment into cloud
        pavFragmentC.pix.left.start = lineStart;
        pavFragmentC.pix.left.end = lineEnd;
        pavFragmentC.pix.right.start = lineStartRight;
        pavFragmentC.pix.right.end = lineEndRight;
        //putPavementFragmentIntoCloud(&pointCloud_msg, &pavFragmentC);
    }
    sidewalkEdges->left.setImgToDetect(&imageResult);
    sidewalkEdges->right.setImgToDetect(&imageResult);
    sidewalkEdges->left.validateEdge(params, pavementCenter);
    sidewalkEdges->right.validateEdge(params, pavementCenter);

    if (params->displayRecognized.orig)
    {
        sidewalkEdges->left.drawAllEdges(imageOrig, params);
        sidewalkEdges->right.drawAllEdges(imageOrig, params);
        sidewalkEdges->left.drawDetectedPoints(imageOrig, params);
        sidewalkEdges->right.drawDetectedPoints(imageOrig, params);
    }
    if (params->displayRecognized.result)
    {
        sidewalkEdges->left.drawAllEdges(&imageResult, params);
        sidewalkEdges->right.drawAllEdges(&imageResult, params);
        sidewalkEdges->left.drawDetectedPoints(&imageResult, params);
        sidewalkEdges->right.drawDetectedPoints(&imageResult, params);
    }
    /*for (int i = 0; i < params->calibrationPoints.size(); i++)
    {
        circle(*imageOrig, params->calibrationPoints[i], 5, cv::Scalar(255, 255, 255), -1, 8);
    }*/



    *imageResultOut = imageResult;

    return isValid;
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