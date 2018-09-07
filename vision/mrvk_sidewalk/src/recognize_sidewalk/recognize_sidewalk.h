#ifndef PROJECT_RECOGNIZESIDEWALK_H
#define PROJECT_RECOGNIZESIDEWALK_H

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
//#include "pavement_to_marker/pavement_to_marker.h"
//#include "pavement_to_cloud/pavement_to_cloud.h"

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "../RecognizeSidewalkParams.h"
#include "SidewalkEdge.h"

short recognize_sidewalk_frame(cv::Mat *imageOrig, cv::Mat *imageResultOut, RecognizeSidewalkParams *params, SidewalkEdges *sidewalkEdges, ros::NodeHandle n);
int getLeftPavementPoint(cv::Mat image, int line, int pavementCenter, int edge_side_offset_promile);
int getRightPavementPoint(cv::Mat image, int line, int pavementCenter, int edge_side_offset_promile);
int getPavementCenter(int leftPoint, int rightPoint, int pavementCenterLast);
bool isOpeningLeft(int startPoint, int endPoint, int sideOffset);
bool isOpeningRight(int imgCols, int startPoint, int endPoint, int sideOffset);
bool notPavement(int startPoint, int endPoint, int pavementCenter, int sideOffset);

#endif //PROJECT_RECOGNIZESIDEWALK_H