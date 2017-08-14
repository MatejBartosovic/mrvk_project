#ifndef PROJECT_RECOGNIZESIDEWALKPARAMS_H
#define PROJECT_RECOGNIZESIDEWALKPARAMS_H

//ros
#include <ros/ros.h>

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

    int getParametersFromServer(ros::NodeHandle n);
};

#endif //PROJECT_RECOGNIZESIDEWALKPARAMS_H
