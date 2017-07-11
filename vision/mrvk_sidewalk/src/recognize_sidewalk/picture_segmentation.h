#ifndef PROJECT_PICTURE_SEGMENTATION_H
#define PROJECT_PICTURE_SEGMENTATION_H

#define dilate_size 10
#define erode_size 10

cv::Mat picture_segmentation_frame(cv::Mat frame);
cv::Mat picture_segmentation_frame_HSV(cv::Mat frame);


#endif //PROJECT_PICTURE_SEGMENTATION_H
