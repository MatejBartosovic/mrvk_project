#include "SidewalkEdge.h"
#define COLOR_EDGE_RAW cv::Scalar(128, 0, 128) //purple
#define COLOR_EDGE_VALID cv::Scalar(0, 255, 0) //green
#define COLOR_EDGE_FIX cv::Scalar(0, 255, 255) //yellow
#define COLOR_POINTS_OPTICAL_FLOW cv::Scalar(0, 255, 255) //orange
#define COOLOR_POINTS_DETECTED cv::Scalar(0, 255, 255) //blue

Edge::Edge()
{

}
Edge::~Edge()
{

}
void Edge::computeLineEquation()
{
    for (int i = 0; i < edge.size(); i++)
    {
        lineEquation lineEquationC;
        if (edge.at(i).end.x == edge.at(i).start.x)
        {
            ///perpendicular line
            edge.at(i).myLineEquation.slope = 0;
            edge.at(i).myLineEquation.yIntercept = 0;
        }
        else
        {
            edge.at(i).myLineEquation.slope = (double)(edge.at(i).end.y - edge.at(i).start.y)/(edge.at(i).end.x - edge.at(i).start.x);
            edge.at(i).myLineEquation.yIntercept = edge.at(i).start.y - (double)(edge.at(i).myLineEquation.slope*edge.at(i).start.x);
        }
    }
}
void Edge::new_line()
{
    LineStructure newLine;
    edge.push_back(newLine);
}
void Edge::drawEdge(cv::Mat *img, const cv::Scalar& color, RecognizeSidewalkParams *params)
{
    for (int i = 0; i < edge.size(); i++)
    {
        if (edge.at(i).valid)
        {
            line(*img, edge.at(i).start, edge.at(i).end, color, params->edge_marker_width, EDGE_MARKER_TYPE, EDGE_MARKER_SHIFT);
        }
    }
}
std::vector<LineStructure> *Edge::getEdge()
{
    return &edge;
}

SidewalkEdge::SidewalkEdge()
{

}
SidewalkEdge::~SidewalkEdge()
{

}
void SidewalkEdge::validateEdge()
{

}
void SidewalkEdge::fixEdge()
{

}
void SidewalkEdge::computeLineEquation()
{
    edgeFix.computeLineEquation();
    edgeValid.computeLineEquation();
    edgeRaw.computeLineEquation();
}
void SidewalkEdge::new_line()
{
    edgeRaw.new_line();
    edgeValid.new_line();
    edgeFix.new_line();
}
void SidewalkEdge::clearEdge()
{
    if (!edgeOld.getEdge()->empty())
    {
        *edgeOld.getEdge() = *edgeFix.getEdge();
    }
    edgeRaw.getEdge()->clear();
    edgeValid.getEdge()->clear();
    edgeFix.getEdge()->clear();
}
void SidewalkEdge::drawEdgeRaw(cv::Mat *img, RecognizeSidewalkParams *params)
{
    edgeRaw.drawEdge(img, COLOR_EDGE_RAW, params);
}
void SidewalkEdge::drawEdgeValid(cv::Mat *img, RecognizeSidewalkParams *params)
{
    edgeValid.drawEdge(img, COLOR_EDGE_VALID, params);
}
void SidewalkEdge::drawEdgeFix(cv::Mat *img, RecognizeSidewalkParams *params)
{
    edgeFix.drawEdge(img, COLOR_EDGE_FIX, params);
}
void SidewalkEdge::drawAllEdges(cv::Mat *img, RecognizeSidewalkParams *params)
{
    edgeRaw.drawEdge(img, COLOR_EDGE_RAW, params);
    edgeFix.drawEdge(img, COLOR_EDGE_FIX, params);
    edgeValid.drawEdge(img, COLOR_EDGE_VALID, params);
}

std::vector<LineStructure> *SidewalkEdge::getEdgeRaw()
{
    return edgeRaw.getEdge();
}
std::vector<LineStructure> *SidewalkEdge::getEdgeValid()
{
    return edgeValid.getEdge();
}
std::vector<LineStructure> *SidewalkEdge::getEdgeFix()
{
    return edgeFix.getEdge();
}

