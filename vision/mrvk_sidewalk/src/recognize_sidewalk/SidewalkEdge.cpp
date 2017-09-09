#include "SidewalkEdge.h"
#include "../misc_tools/misc_tools.h"
#include <unistd.h>
#include <math.h>

#define COLOR_EDGE_RAW cv::Scalar(128, 0, 128) //purple
#define COLOR_EDGE_VALID cv::Scalar(0, 255, 0) //green
#define COLOR_EDGE_FIX cv::Scalar(255, 255, 0) //yellow
#define COLOR_POINTS_OPTICAL_FLOW cv::Scalar(0, 140, 255) //orange
#define COLOR_POINTS_DETECTED cv::Scalar(255, 0, 0) //blue

#define M_PI 3.14159265358979323846

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
            edge.at(i).myLineEquation.slope = std::numeric_limits<double>::max();
            edge.at(i).myLineEquation.yIntercept = edge.at(i).start.y - (double)(edge.at(i).myLineEquation.slope*edge.at(i).start.x);
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
void SidewalkEdge::validateEdge(RecognizeSidewalkParams *params, int pavementCenter)
{
    computeOpticalFlow(&newImg, &oldImg, &oldPoints, &newPoints);
    int sideOffset = (params->edge_side_offset_promile*newImg.cols)/1000;
    *getEdgeValid() = *getEdgeRaw();
    getSidewalkArea(&newImg, params);
    computeLineEquation();

    validPoints.clear();
    for (int i = 0; i < getEdgeRaw()->size(); i++)
    {
        getEdgeValid()->at(i).valid = false;
        if (!glitchedFrame())
        {
            if (!isOpening(newImg.cols, getEdgeRaw()->at(i).start.x, getEdgeRaw()->at(i).end.x, params->sideOffest))
            {
                if (!notPavement(getEdgeRaw()->at(i).start.x, getEdgeRaw()->at(i).end.x, pavementCenter, sideOffset))
                {
                    getEdgeValid()->at(i).valid = true;
                    validPoints.push_back(getEdgeValid()->at(i).start);
                    //fillLineWithPoints(&validPoints, getEdgeValid()->at(i).myLineEquation, getEdgeValid()->at(i).start, getEdgeValid()->at(i).end);
                }
            }
        }
        else
        {
            //todo try to repair glitched frame - leave for last
        }
    }
    slopeValidate(params);
    for (int i = 0; i < getEdgeValid()->size(); i++)
    {
        if (getEdgeValid()->at(i).valid)
        {
            getEdgeValid()->at(i).valid = false;
            if (!glitchedFrameSlope())
            {
                getEdgeValid()->at(i).valid = true;
            }
            else
            {
                //todo try to repair glitched frame - leave for last
            }
        }
    }

}
void SidewalkEdge::fillLineWithPoints(std::vector<cv::Point> *outFilledPointsEdge, lineEquation lineEquationC, cv::Point lineCmStart, cv::Point lineCmEnd)
{
    //point cloud
    cv::Point pavPoint;
    pavPoint.x = lineCmStart.x;
    pavPoint.y = lineCmStart.y;
    pointCm pointCm1;
    pointCm1.x = lineCmStart.x;
    pointCm1.y = lineCmStart.y;
    pointCm1.z = DEFAULT_PAV_Z;

    double lineDirection = 1.0;
    if (lineCmStart.x > lineCmEnd.x)
    {
        lineDirection = -1.0;
    }

    while ( (fabs(pointCm1.y - lineCmEnd.y) > m2cm(PAV_LINE_RESOLUTION))||(fabs(pointCm1.x - lineCmEnd.x) > m2cm(PAV_LINE_RESOLUTION)) )
    {
        if (lineCmEnd.x == lineCmStart.x)
        {
            pointCm1.x = lineCmEnd.x;
            pointCm1.y = pointCm1.y - m2cm(PAV_LINE_RESOLUTION);
        }
        else
        {
            if ((B_KVADR*B_KVADR - 4.0*A_KVADR*C_KVADR) == 0)
            {
                pointCm1.x = -B_KVADR/2/A_KVADR;
                pointCm1.y = pointCm1.x*lineEquationC.slope + lineEquationC.yIntercept;
                pavPoint.x = cm2m(pointCm1.x);
                pavPoint.y = cm2m(pointCm1.y);
                outFilledPointsEdge->push_back(pavPoint);
                if (outFilledPointsEdge->size() > MAX_NUM_POINTS_POINTCLOUD)
                {
                    ROS_ERROR("Point count overload!");
                    break;
                }
            }
            else if ((B_KVADR*B_KVADR - 4.0*A_KVADR*C_KVADR) > 0)
            {
                pointCm1.x = (-B_KVADR + lineDirection*sqrt(B_KVADR*B_KVADR - 4.0*A_KVADR*C_KVADR))/2/A_KVADR;

                pointCm1.y = pointCm1.x*lineEquationC.slope + lineEquationC.yIntercept;
                pavPoint.x = cm2m(pointCm1.x);
                pavPoint.y = cm2m(pointCm1.y);
                outFilledPointsEdge->push_back(pavPoint);
                if (outFilledPointsEdge->size() > MAX_NUM_POINTS_POINTCLOUD)
                {
                    ROS_ERROR("Point count overload!");
                    break;
                }
            }
            else
            {
                ROS_ERROR("Couldn't put line point to cloud!");
                break;
            }
        }
    }
}
int SidewalkEdge::computeSlopeMedian(RecognizeSidewalkParams *params) {
    int maxNumMedianSlope = 10;
    int numPoints = 0;
    int lineAngle = 0;
    std::vector<double> slopeVector;
    for (int i = 0; i < getEdgeValid()->size(); i++) {
        if (getEdgeValid()->at(i).valid) {
            lineAngle = atan(getEdgeValid()->at(i).myLineEquation.slope) / M_PI * 180;
            if (lineAngle < 0) {
                lineAngle = lineAngle + 180;
            }
            slopeVector.push_back(lineAngle);
            numPoints++;
        }
    }
    medianSlopeFrame = doubleMedian(slopeVector);

    //detect frame with high variance
    //A = 1:10
    //A = [2 2 2 2 2 1 1 1 1 5 ]
    //meanA = mean(A)
    //variance = mean(abs((A - meanA).*2))
    double variance = 0;
    for (int i = 0; i < slopeVector.size(); i++) {
        variance += fabs(slopeVector.at(i) - medianSlopeFrame) * 2;
    }
    if (slopeVector.size() != 0) {
        variance = variance / slopeVector.size();
    } else {
        variance = std::numeric_limits<double>::max();
    }
    glitchedFrameSlopeVal = false;
    if (variance > params->glitchFrame.max_slope_variance)
    {
        glitchedFrameSlopeVal = true;
    }
    //ROS_ERROR("Variance = %lf", variance);
    return numPoints;
}
void SidewalkEdge::slopeValidate(RecognizeSidewalkParams *params)
{
    invalidFrame = false;
    //glitchedFrameSlopeVal = false;
    perpendicularSidewalk = isSidewalkPerpendicular(params);
    if (!computeSlopeMedian(params)) return;
    if (medianSlope.size() >= 10)
    {
        //ROS_ERROR("Angle deviation %lf", fabs(doubleMedian(medianSlope) - medianSlopeFrame));
        if (fabs(doubleMedian(medianSlope) - medianSlopeFrame) < params->glitchFrame.max_slope_deviation)
        {

            medianSlope.push_back(medianSlopeFrame);
            medianSlope.erase(medianSlope.begin());
        }
        else
        {
            invalidFrame = true;
            //glitchedFrameSlopeVal = true;
        }
    }
    else
    {
        medianSlope.push_back(medianSlopeFrame);
    }
    //TODO validate single lines
}
bool SidewalkEdge::isSidewalkPerpendicular(RecognizeSidewalkParams *params)
{
    bool perpendicular = false;
    double slopePerpendicularSpread = 10;
    //ROS_ERROR("Slope median %lf", medianSlopeFrame);
    if ((medianSlopeFrame > (180 - params->glitchFrame.slope_perpendicular_spread))||(medianSlopeFrame < (params->glitchFrame.slope_perpendicular_spread)))
    {
        perpendicular = true;
        //ROS_ERROR("Perpendicular motherfucker!");
    }
    return perpendicular;
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
bool SidewalkEdge::isOpening(int imgCols, int startPoint, int endPoint, int sideOffset)
{
    bool opening = false;
    if ((startPoint > (imgCols - sideOffset))&&(endPoint > (imgCols - sideOffset)))
    {
        opening = true;
    }
    if ((startPoint < sideOffset)&&(endPoint < sideOffset))
    {
        opening = true;
    }
    return opening;
}
bool SidewalkEdge::glitchedFrame()
{
    return glitchedFrameVal;
}
bool SidewalkEdge::glitchedFrameSlope()
{
    return glitchedFrameSlopeVal;
}
void SidewalkEdge::detectFrameGlitch(RecognizeSidewalkParams *params)
{
    /*if (glitchedFrameVal)
    {
        getchar();
    }*/
    glitchedFrameVal = false;
    if (sidewalkArea.size() >= params->glitchFrame.areaBufferSize)
    {
        if (fabs(intMedian(sidewalkArea) - sidewalkAreaFrame) > params->glitchFrame.maxAreaDifference)
        {
            glitchedFrameVal = true;
        }
    }
}
void SidewalkEdge::getSidewalkArea(cv::Mat *img, RecognizeSidewalkParams *params)
{
    sidewalkAreaFrame = 0;
    for (int x = 0; x < img->cols; x += params->glitchFrame.webSize)
    {
        for (int y = 0; y < img->rows; y += params->glitchFrame.webSize)
        {
            if (img->at<cv::Vec3b>(x, y)[0] != 0)
            {
                sidewalkAreaFrame++;
            }
        }
    }
    detectFrameGlitch(params);
    if (!glitchedFrame())
    {
        sidewalkArea.push_back(sidewalkAreaFrame);
    }
    if (params->glitchFrame.areaBufferSize < sidewalkArea.size())
    {
        sidewalkArea.erase(sidewalkArea.begin());
    }
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
    if (params->displayRecognized.raw)
    {
        edgeRaw.drawEdge(img, COLOR_EDGE_RAW, params);
    }
    if (params->displayRecognized.valid)
    {
        edgeFix.drawEdge(img, COLOR_EDGE_FIX, params);
    }
    if (params->displayRecognized.valid)
    {
        edgeValid.drawEdge(img, COLOR_EDGE_VALID, params);
    }
}
void SidewalkEdge::drawDetectedPoints(cv::Mat *img, RecognizeSidewalkParams *params)
{
    //show points
    if (!newPoints.empty()&&params->displayRecognized.newPts)
    {
        size_t i, k;
        for (i = 0; i < newPoints.size(); i++)
        {
            circle(*img, newPoints[i], 5, COLOR_POINTS_OPTICAL_FLOW, -1, 8);
        }
    }
    if (!oldPoints.empty()&&params->displayRecognized.oldPts)
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

//filters lines which are incorrect because detected point was detected in pavement center
bool notPavement(int startPoint, int endPoint, int pavementCenter, int sideOffset)
{
    bool notPavement = false;
    if ((startPoint >= pavementCenter - sideOffset-10)&&(startPoint <= pavementCenter + sideOffset+10))
    {
        notPavement = true;
    }
    if ((endPoint >= pavementCenter - sideOffset-10)&&(endPoint <= pavementCenter + sideOffset+10))
    {
        notPavement = true;
    }
    return notPavement;
}

//TODO zhustit siet detekovanych bodov (pri ciarach bliziacich sa vodorovnym sa stratia ciary kvoli nedostatky bodov) ak sa dorobi detekcia vodorovnych ciar, tak sa da pocet detekovanych bodov do povodneho stavu