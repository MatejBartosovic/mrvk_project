#ifndef PROJECT_RECOGNIZESIDEWALKPARAMS_H
#define PROJECT_RECOGNIZESIDEWALKPARAMS_H

//ros
#include <ros/ros.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct GlitchFrame{
    int areaBufferSize = 6;
    int maxAreaDifference = 100;
    int webSize = 30;
    double max_slope_deviation = 20;
    double slope_perpendicular_spread = 10;
    double max_slope_variance = 90;
};

struct DisplayRecognized{
    bool raw = true;
    bool valid = true;
    bool fix = true;
    bool newPts = true;
    bool oldPts = true;
    bool orig = true;
    bool result = true;
};

class RecognizeSidewalkParams
{
private:
public:
    int ros_parameter = 0;
    std::string image_topic = "/my_kinect/hd/image_color";
    std::string depth_image_topic = "/kinect2/qhd/image_depth_rect";
    int spinFreq = 20;

    int sideOffest = 1;
    int edge_marker_width = 6;
    int edge_points_dist = 100;
    int edge_start_offset = 50;
    int edge_side_offset_promile = 10;
    int detect_percent_of_image = 80;

    std::vector<cv::Point> calibrationPoints;
    DisplayRecognized displayRecognized;
    GlitchFrame glitchFrame;

    int getParametersFromServer(ros::NodeHandle n);
    int getCalibParametersFromServer(ros::NodeHandle n);
    int getDisplayRecognized(ros::NodeHandle n);
    int getGlitchFrame(ros::NodeHandle n);
};

#endif //PROJECT_RECOGNIZESIDEWALKPARAMS_H
