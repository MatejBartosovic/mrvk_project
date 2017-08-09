#include "RecognizeSidewalkParams.h"
#include <string>

int RecognizeSidewalkParams::getParametersFromServer(ros::NodeHandle n)
{
    n.getParam("sidewalk_params/my_param", ros_parameter);
    n.getParam("sidewalk_params/image_topic", image_topic);
    n.getParam("sidewalk_params/depth_image_topic", depth_image_topic);
    n.getParam("sidewalk_params/spin_freq", spinFreq);

    n.getParam("sidewalk_params/edge_side_offset", sideOffest);
    n.getParam("sidewalk_params/edge_marker_width", edge_marker_width);
    n.getParam("sidewalk_params/edge_start_offset", edge_start_offset);
    n.getParam("sidewalk_params/edge_side_offset_promile", edge_side_offset_promile);
    n.getParam("sidewalk_params/detect_percent_of_image", detect_percent_of_image);

    n.getParam("sidewalk_params/edge_points_distribution/edge_points_dist", edge_points_dist);
    getCalibParametersFromServer(n);

    return 0;
}

int RecognizeSidewalkParams::getCalibParametersFromServer(ros::NodeHandle n)
{
    std::string parameterName;
    int pointIter = 0;
    int readX = 0;
    int readY = 0;
    parameterName = "sidewalk_transform_calib/point_" + std::to_string(pointIter) + "_x";
    while (n.getParam(parameterName.c_str(), readX))
    {
        n.getParam(parameterName.c_str(), readY);
        calibrationPoints = cv::Point(readX, readY);
        pointIter++;
    }

    return 0;
}