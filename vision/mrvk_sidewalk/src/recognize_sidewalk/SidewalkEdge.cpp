#include "SidewalkEdge.h"
#define COLOR_EDGE_RAW cv::Scalar(128, 0, 128) //purple
#define COLOR_EDGE_VALID cv::Scalar(0, 255, 0) //green
#define COLOR_EDGE_FIX cv::Scalar(255, 255, 0) //yellow
#define COLOR_POINTS_OPTICAL_FLOW cv::Scalar(0, 140, 255) //orange
#define COLOR_POINTS_DETECTED cv::Scalar(255, 0, 0) //blue

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
    computeOpticalFlow(&newImg, &oldImg, &oldPoints, &newPoints);
    //for (int i = 0; i < getEdgeRaw(). )
    /*if (!isOpeningLeft(lineStart.x, lineEnd.x, params.sideOffest))
    {
        if (!notPavement(lineStart.x, lineEnd.x, pavementCenter, sideOffset))
        {

        }
    }*/
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
int SidewalkEdge::computeOpticalFlow(cv::Mat *gray, cv::Mat *prevGray, std::vector<cv::Point2f> *points, std::vector<cv::Point2f> *prevPoints)
{
    rawLinesToPoints();
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
    cv::Size subPixWinSize(10, 10), winSize(31, 31);

    const int MAX_COUNT = 500;

    if (!points->empty()) {
        std::vector<uchar> status;
        std::vector<float> err;
        if (prevGray->empty())
            gray->copyTo(*prevGray);
        cv::calcOpticalFlowPyrLK(*prevGray, *gray, *points, *prevPoints, status, err, winSize,
                             3, termcrit, 0, 0.001);
        std::cout << "prevPoints" << prevPoints->size() << std::endl;
    }

    std::swap(*prevPoints, *points);
    //cv::swap(*prevGray, *gray);

    return 0;
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
void SidewalkEdge::setImgToDetect(cv::Mat *img)
{
    if (newImg.empty())
    {
        cv::cvtColor(*img, oldImg, cv::COLOR_BGR2GRAY);
    }
    else
    {
        oldImg = newImg.clone();
    }
    cv::cvtColor(*img, newImg, cv::COLOR_BGR2GRAY);
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
void SidewalkEdge::drawDetectedPoints(cv::Mat *img)
{
    //show points
    if (!newPoints.empty())
    {
        size_t i, k;
        for (i = 0; i < newPoints.size(); i++)
        {
            circle(*img, newPoints[i], 5, COLOR_POINTS_OPTICAL_FLOW, -1, 8);
        }
    }
    if (!oldPoints.empty())
    {
        size_t i, k;
        for (i = 0; i < oldPoints.size(); i++)
        {
            circle(*img, oldPoints[i], 5, COLOR_POINTS_DETECTED, -1, 8);
        }
    }
}
void SidewalkEdge::rawLinesToPoints()
{
    oldPoints.clear();
    for (int i = 0; i < edgeRaw.getEdge()->size(); i++)
    {
        oldPoints.push_back(edgeRaw.getEdge()->at(i).start);
    }
    oldPoints.push_back(edgeRaw.getEdge()->at(edgeRaw.getEdge()->size() - 1).end);
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

