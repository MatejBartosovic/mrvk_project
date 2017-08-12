#ifndef PROJECT_PICTURE_SEGMENTATION_H
#define PROJECT_PICTURE_SEGMENTATION_H

#include"segment_class.hpp"



cv::Mat picture_segmentation_frame(cv::Mat frame);
cv::Mat picture_segmentation_frame_HSV(cv::Mat frame);
cv::Mat picture_segmentation_frame_c1c2c3(cv::Mat frame);

cv::Vec3b computeAdaptationKernels(cv::Mat imageHSV);
cv::Mat maskExposure(cv::Mat unmasked_image);
cluster extractRegion(cv::Mat unregioned_image);
int setRange(int value, int range);
cv::Mat convertc123(cv::Mat in_imageRGB);
cluster123 extractRegion123(cv::Mat unregioned_image, cv::Mat mask_image);
cv::Mat maskExposure123(cv::Mat unmasked_image, cv::Mat unmasked_imageRGB);
double setRange123(double value, double range, bool flag);

#endif //PROJECT_PICTURE_SEGMENTATION_H
