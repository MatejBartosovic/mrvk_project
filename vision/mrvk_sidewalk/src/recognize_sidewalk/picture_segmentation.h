#ifndef PROJECT_PICTURE_SEGMENTATION_H
#define PROJECT_PICTURE_SEGMENTATION_H

#include"segment_class.hpp"
#include "SidewalkEdge.h"



cv::Mat picture_segmentation_frame(cv::Mat frame);
cv::Mat picture_segmentation_frame_HSV(cv::Mat frame, ros::NodeHandle n);
cv::Mat picture_segmentation_frame_c1c2c3(cv::Mat frame, ros::NodeHandle n);
cv::Mat picture_segmentation_frame_c1c2c3_check(cv::Mat frame, short *valid,  SidewalkEdges *sidewalkEdges, ros::NodeHandle n);


cv::Vec3b computeAdaptationKernels(cv::Mat imageHSV, ros::NodeHandle n);
cv::Mat maskExposure(cv::Mat unmasked_image, ros::NodeHandle n);
cluster extractRegion(cv::Mat unregioned_image);
int setRange(int value, int range);
cv::Mat convertc123(cv::Mat in_imageRGB);
cluster123 extractRegion123(cv::Mat unregioned_image, cv::Mat mask_image);
cv::Mat maskExposure123(cv::Mat unmasked_image, cv::Mat unmasked_imageRGB);
bool improveShadows(cv::Mat iputImgHSV,cv::Mat inputImgRGB, ros::NodeHandle n);
double setRange123(double value, double range, bool flag);
cluster123 updateModel123(cluster123 clustLearnt, cluster123 clustTraining);
double calcPointDistance123(cluster123 clustLearnt, cluster123 clustTrainin);
double calcClusterDistance123(cluster123 clustLearnt, cluster123 clustTraining);

#endif //PROJECT_PICTURE_SEGMENTATION_H
