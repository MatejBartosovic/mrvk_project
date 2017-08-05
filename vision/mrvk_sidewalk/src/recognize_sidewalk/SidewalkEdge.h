#ifndef PROJECT_SIDEWALKEDGE_H
#define PROJECT_SIDEWALKEDGE_H

//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

struct LineStructure{
    cv::Point lineStart;
    cv::Point lineEnd;
    lineEquation myLineEquation;
    bool validity = true;
};
class Edge{
private:
    std::queue<std::vector<LineStructure> > edge;
public:
};

class SidewalkEdge{
private:
    Edge edgeRaw;//purple
    Edge edgeValid;//green
    Edge edgeFix;//yellow
    //todo add line queue for line which replaces invalid lines - display with diferent color (yellow)
    //todo test contours for edge
    //todo optical flow will help us determine if we are turning and whether we are perpendicular to sidewalk
    //todo zda sa, ze ked je presvetleny obraz, tak segmentacia je dobra aspon pri robotovy, budeme teda pri rozpoznani hran toto brat do uvahy (budeme pocitat iba so spodkom obrazu).
public:
    void computeLineEquation();

};
#endif //PROJECT_SIDEWALKEDGE_H
