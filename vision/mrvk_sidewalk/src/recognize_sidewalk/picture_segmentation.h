#ifndef PROJECT_PICTURE_SEGMENTATION_H
#define PROJECT_PICTURE_SEGMENTATION_H



cv::Mat picture_segmentation_frame(cv::Mat frame);
cv::Mat picture_segmentation_frame_HSV(cv::Mat frame);
cv::Vec3b computeAdaptationKernels(cv::Mat imageHSV);


#endif //PROJECT_PICTURE_SEGMENTATION_H
