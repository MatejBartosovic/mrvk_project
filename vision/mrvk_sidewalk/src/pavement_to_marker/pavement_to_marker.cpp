#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

void vypis()
{
    std::cout << "Vypis z kniznice Marker!" << std::endl;
}
void putPavementFragmentIntoMarker(int map[100000],cv::Point startLineLeft, cv::Point endLineLeft, cv::Point startLineRight, cv::Point endLineRight)
{

    //getPavementWidth(startLineLeft, startLineRight)
    //getDistFromImCenter left and right point of pavement
    //change pavement point to real coordinates

    //getPavementWidth(endLineLeft, endLineRight)
    //getDistFromImCenter left and right point of pavement
    //change pavement point to real coordinates

    //putPavementLineIntoMarker(map, leftLineRealCoordStartX,leftLineRealCoordStartY, LeftLineRealCoordEndX, leftLineRealCoordEndY);
    //putPavementLineIntoMarker(map, rightLineRealCoordStartX,rightLineRealCoordStartY, righttLineRealCoordEndX, rightLineRealCoordEndY);

}
void putPavementLineIntoMarker(int map[100000], int lineRealCoordStartX, int lineRealCoordStartY, int lineRealCoordEndX, int lineRealCoordEndY) //real coordinates in mm
{

}