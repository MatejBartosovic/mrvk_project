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
    getDisplayRecognized(n);
    getGlitchFrame(n);

    return 0;
}

int RecognizeSidewalkParams::getCalibParametersFromServer(ros::NodeHandle n)
{
    std::string parameterName;
    int pointIter = 0;
    int readX = 0;
    int readY = 0;
    parameterName = "sidewalk_transform_calib/point_" + std::to_string(pointIter) + "_x";
    cv::Point calibrationPoint;
    while (n.getParam(parameterName.c_str(), readX))
    {
        parameterName = "sidewalk_transform_calib/point_" + std::to_string(pointIter) + "_y";
        n.getParam(parameterName.c_str(), readY);
        calibrationPoint = cv::Point(readX, readY);
        calibrationPoints.push_back(calibrationPoint);
        pointIter++;
        parameterName = "sidewalk_transform_calib/point_" + std::to_string(pointIter) + "_x";
    }

    return 0;
}

int RecognizeSidewalkParams::getDisplayRecognized(ros::NodeHandle n)
{
    n.getParam("sidewalk_params/display/edge/raw", displayRecognized.raw);
    n.getParam("sidewalk_params/display/edge/valid", displayRecognized.valid);
    n.getParam("sidewalk_params/display/edge/fix", displayRecognized.fix);
    n.getParam("sidewalk_params/display/points/new", displayRecognized.newPts);
    n.getParam("sidewalk_params/display/points/old", displayRecognized.oldPts);
    n.getParam("sidewalk_params/display/points/old", displayRecognized.oldPts);
    n.getParam("sidewalk_params/display/orig", displayRecognized.orig);
    n.getParam("sidewalk_params/display/result", displayRecognized.result);
}
int RecognizeSidewalkParams::getGlitchFrame(ros::NodeHandle n)
{
    n.getParam("sidewalk_params/glitchFrame/areaBufferSize", glitchFrame.areaBufferSize);
    n.getParam("sidewalk_params/glitchFrame/maxAreaDifference", glitchFrame.areaBufferSize);
    n.getParam("sidewalk_params/glitchFrame/webSize", glitchFrame.areaBufferSize);
    n.getParam("sidewalk_params/glitchFrame/max_slope_deviation", glitchFrame.max_slope_deviation);
    n.getParam("sidewalk_params/glitchFrame/slope_perpendicular_spread", glitchFrame.slope_perpendicular_spread);
    n.getParam("sidewalk_params/glitchFrame/max_slope_variance", glitchFrame.max_slope_variance);
}