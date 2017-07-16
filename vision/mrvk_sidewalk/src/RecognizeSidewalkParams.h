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
    int sideOffest = 1;
    int spinFreq = 20;
    int edge_marker_width = 6;

    int getParametersFromServer(ros::NodeHandle n);
};

#endif //PROJECT_RECOGNIZESIDEWALKPARAMS_H
