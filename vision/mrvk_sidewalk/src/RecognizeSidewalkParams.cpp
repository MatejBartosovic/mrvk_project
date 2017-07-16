#include "RecognizeSidewalkParams.h"

int RecognizeSidewalkParams::getParametersFromServer(ros::NodeHandle n)
{
    n.getParam("sidewalk_params/my_param", ros_parameter);
    n.getParam("sidewalk_params/image_topic", image_topic);
    n.getParam("sidewalk_params/side_offset", sideOffest);
    n.getParam("sidewalk_params/spin_freq", spinFreq);
    n.getParam("sidewalk_params/edge_marker_width", edge_marker_width);
    return 0;
}