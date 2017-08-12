#ifndef PROJECT_PICTURE_SEGMENTATION_H
#define PROJECT_PICTURE_SEGMENTATION_H

#include"segment_class.hpp"



cv::Mat picture_segmentation_frame(cv::Mat frame);
cv::Mat picture_segmentation_frame_HSV(cv::Mat frame);
cv::Vec3b computeAdaptationKernels(cv::Mat imageHSV);
cv::Mat maskExposure(cv::Mat unmasked_image);
cluster extractRegion(cv::Mat unregioned_image);
int setRange(int value, int range);



#endif //PROJECT_PICTURE_SEGMENTATION_H
