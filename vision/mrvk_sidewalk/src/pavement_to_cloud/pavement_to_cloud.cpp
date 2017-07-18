#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <math.h>

#include <sensor_msgs/PointCloud.h>
#include "pavement_to_cloud.h"

#define PI (3.141592653589793)

void putPavementFragmentIntoCloud(sensor_msgs::PointCloud *pointCloud_msg, pavFragment *pavFragmentC)///Done //cv::Point startLineLeft, cv::Point endLineLeft, cv::Point startLineRight, cv::Point endLineRight, pointCm startLineCmLeft, pointCm startLineCmRight)
{
    //pointCm startLineCmLeft;
    //pointCm startLineCmRight;
    //pointCm endLineCmLeft;
    //pointCm endLineCmRight;

    //start points  - treba Y zobrat z predchadzajuceho cyklu, bude vchadzat do funkcie ako smernik
    //changePavPointToCm(startLineLeft, startLineRight, &startLineCmLeft, &startLineCmRight);
    //changePavPointToCm(startLineLeft, startLineRight)
        //getPavementWidth(startLineLeft, startLineRight)
        //getCenterOfPavement(startLineLeft, startLineRight)
        //compute

    //end points
    changePavPointsToCm(pavFragmentC);//startLineLeft, endLineLeft, startLineRight, endLineRight, &endLineCmLeft, &endLineCmRight, startLineCmLeft, startLineCmRight);
    //change_pav_point_to_Cm(endLineLeft, endLineRight)
        //getPavementWidth(endLineLeft, endLineRight)
        //getCenterOfPavement(endLineLeft, endLineRight)
        //compute

    //left line
    putPavementLineIntoCloud(pointCloud_msg, pavFragmentC->cm.left.start, pavFragmentC->cm.left.end);// startLineCmLeft, endLineCmLeft); //real coordinates in mm
    //putPavementLineIntoCloud(pointCloud, leftLineRealCoordStartX,leftLineRealCoordStartY, LeftLineRealCoordEndX, leftLineRealCoordEndY);
        //leftLineSlope = computeLineSlope(startLeftLine, endLeftLine)
        //putPointsOfLineToCloud(pointCloud, lineSlope, startLeftLine, endLeftLine)

    //right line
    putPavementLineIntoCloud(pointCloud_msg, pavFragmentC->cm.right.start, pavFragmentC->cm.right.end);//startLineCmRight, endLineCmRight);
    //putPavementLineIntoCloud(pointCloud, rightLineRealCoordStartX,rightLineRealCoordStartY, rightLineRealCoordEndX, rightLineRealCoordEndY);
        //rightLineSlope = computeLineSlope(startRightLine, endRightLine)
        //putPointsOfLineToCloud(pointCloud, lineSlope, startRightLine, endRightLine)

    pavFragmentC->cm.left.start = pavFragmentC->cm.left.end;
    pavFragmentC->cm.right.start = pavFragmentC->cm.right.end;

    // pri prepocte bodov na perspektivu sa bude detekovat, ci je bod na okraji obrazku, teda ci je odbocka a ak je,
    // tak sa dany bodo potom nezobrazi ale na vypocet paralelneho bodu sa pouzit moze.
}
void changePavPointsToCm(pavFragment *pavFragmentC)///done //cv::Point endLineLeft, cv::Point startLineRight, cv::Point endLineRight, pointCm *endLineCmLeft, pointCm *endLineCmRight, pointCm startLineCmLeft, pointCm startLineCmRight)
{
    changeToCmX(pavFragmentC);
    changeToCmY(pavFragmentC);


    //stare
    //getPavementWidth(startLineLeft, startLineRight)
    //getPavementWidthCm(int pavementWidthPix)
    //getCenterOfPavement(startLineLeft, startLineRight)
    //compute

    //nove
    //ziskanie x suradnic cm
        //chodnik sirka pix
        //chodnik sirka cm
        //ziskaj stredovy bod chodnika pix
        //chodnik stred pix od stredu obr
        //chodnik stred cm od stredu smerovania
        //ziskaj stredovy bod chodnika cm
        //suradnica x lavy, pravy bod cm
    //ziskanie y suradnic cm
        //prepocet video rozlisenie na kalibracne rozlisenie
        //vypocet bodov ciary v priestore na pomyslenej ploche obrazu - cez cos, sin
        //vypocet priamok prechadzajucich bodmi na obraze
        //Vypocet realnych bodov v cm na urovni vysky 0 cm - priesecnik vypocitanych priamok s osou x

        //dlzka fragmentu chodnika pix
        //dlzka fragmentu chodnika cm - od start pointu cm
        //suradnica y lavy, pravy bod cm
}
int getWidthPix(cv::Point pointLeft, cv::Point pointRight) ///Done
{
    int width = 0;
    width = pointRight.x - pointLeft.x;
    return width;
}
int getWidthCm(int widthPix, int heightPix) ///Done
{
    heightPix = heightPix*PIC_CALIB_HEIGHT_PIX/PIC_HEIGHT_PIX;
    widthPix = widthPix*PIC_CALIB_WIDTH_PIX/PIC_WIDTH_PIX;
    double pixWidthCm = CALIB_OBJ_WIDTH_MM/10.0/(heightPix*WIDTH_K + WIDTH_Q);
    int width = widthPix*pixWidthCm;
    return width;
}
cv::Point getCenterOfPavement(cv::Point pointLeft, cv::Point pointRight)///Done
{
    cv::Point center;
    center.y = pointLeft.y;
    center.x = (pointRight.x - pointLeft.x )/2 + pointLeft.x;
    return center;
}
pointCm getCenterOfPavementCm(int pavCenterFromImgCenterCm, cv::Point pavementCenterPix)///Done
{
    pointCm pavementCenter;
    if ((PIC_WIDTH_PIX/2) < pavementCenterPix.x)
    {
        pavementCenter.x = pavCenterFromImgCenterCm;
    }
    else
    {
        pavementCenter.x = -pavCenterFromImgCenterCm;
    }

    return pavementCenter;
}


int computeCmX(pointCm pavCenterCm, int pavementWidthCm, bool isLeft)///done
{
    int coordX = 0;
    if (isLeft == true)
    {
        coordX = pavCenterCm.x - pavementWidthCm/2;
    }
    else
    {
        coordX = pavCenterCm.x + pavementWidthCm/2;
    }
    return coordX;
}

void changeToCmX(pavFragment *pavFragmentC)
{
    ///stale nepocitam sirku chodnika v start bodoch
    int pavementWidthPix = getWidthPix(pavFragmentC->pix.left.end, pavFragmentC->pix.right.end);// endLineLeft, endLineRight);
    int pavementWidthCm = getWidthCm(pavementWidthPix, pavFragmentC->pix.right.end.y);
    cv::Point pavCenterPix = getCenterOfPavement(pavFragmentC->pix.left.end, pavFragmentC->pix.right.end);//endLineLeft, endLineRight);
    cv::Point imgCenterPoint;
    imgCenterPoint.x = PIC_WIDTH_PIX/2;
    imgCenterPoint.y = PIC_HEIGHT_PIX/2;
    int pavCenterFromImgCenterPix = getWidthPix(imgCenterPoint, pavCenterPix);
    int pavCenterFromImgCenterCm = getWidthCm(abs(pavCenterFromImgCenterPix), pavFragmentC->pix.left.end.y);
    pointCm pavCenterCm = getCenterOfPavementCm(pavCenterFromImgCenterCm, pavCenterPix);
    pavFragmentC->cm.left.end.x = computeCmX(pavCenterCm, pavementWidthCm, true);
    pavFragmentC->cm.right.end.x = computeCmX(pavCenterCm, pavementWidthCm, false);
    ///pridat vypocet start X
}
int getFragmentLengthCmY(int startLineY, int endLineY, pointCm *startLine, pointCm *endLine) ///treba prerobit, teda netreba pocitat lengthPix ale pocitat pomocou rovnic priamky
{
    int length = 0;
    //prepocet video rozlisenie na kalibracne rozlisenie, treba invertovat hodnotu kvoli suradnej sustave obrazu
    startLineY = PIC_CALIB_HEIGHT_PIX - (startLineY*PIC_CALIB_HEIGHT_PIX/PIC_HEIGHT_PIX);
    endLineY = PIC_CALIB_HEIGHT_PIX - (endLineY*PIC_CALIB_HEIGHT_PIX/PIC_HEIGHT_PIX);

    //vypocet bodov ciary v priestore na pomyslenej ploche obrazu - cez cos, sin
    pointCm startPoint;
    pointCm endPoint;
    startPoint.x = cos(CAMERA_ANGLE/180.0*PI)*startLineY + IMG_OFFSET;
    startPoint.y = sin(CAMERA_ANGLE/180.0*PI)*startLineY;
    endPoint.x = cos(CAMERA_ANGLE/180.0*PI)*endLineY + IMG_OFFSET;
    endPoint.y = sin(CAMERA_ANGLE/180.0*PI)*endLineY;

    //vypocet priamok prechadzajucich bodmi na obraze
    pointCm cameraPoint;
    cameraPoint.z = 0;
    cameraPoint.x = CAMERA_POSE_X;
    cameraPoint.y = CAMERA_POSE_Y;
    lineEquation lineEquationStartPt = computeLineSlope(cameraPoint, startPoint);
    lineEquation lineEquationEndPt = computeLineSlope(cameraPoint, endPoint);
    //Vypocet realnych bodov v cm na urovni vysky 0 cm - priesecnik vypocitanych priamok s osou x
    startPoint.x = -lineEquationStartPt.yIntercept/lineEquationStartPt.slope;
    endPoint.x = -lineEquationEndPt.yIntercept/lineEquationEndPt.slope;
    length = abs(startPoint.x - endPoint.x)/10; //10 from mm to cm
    startLine->y = startPoint.x/10.0;//10 from mm to cm
    endLine->y = endPoint.x/10.0;//10 from mm to cm
    return length;
}
void changeToCmY(pavFragment *pavFragmentC)
{
    int fragmentLengthCmLeftY = getFragmentLengthCmY(pavFragmentC->pix.left.start.y, pavFragmentC->pix.left.end.y, &(pavFragmentC->cm.left.start), &(pavFragmentC->cm.left.end));
    int fragmentLengthCmRightY = getFragmentLengthCmY(pavFragmentC->pix.right.start.y, pavFragmentC->pix.right.end.y, &(pavFragmentC->cm.right.start), &(pavFragmentC->cm.right.end));

    /*pavFragmentC->cm.left.end.y = -fragmentLengthCmLeftY + pavFragmentC->cm.left.start.y/1000;
    ROS_ERROR("OOOO %d", pavFragmentC->cm.left.end.y);
    pavFragmentC->cm.right.end.y = -fragmentLengthCmRightY + pavFragmentC->cm.right.start.y/1000;
    ROS_ERROR("OOOO %d", pavFragmentC->cm.right.end.y );*/
}


void putPavementLineIntoCloud(sensor_msgs::PointCloud *pointCloud_msg, pointCm lineCmStart, pointCm lineCmEnd)///Done //real coordinates in mm
{
    lineEquation lineEquationC = computeLineSlope(lineCmStart, lineCmEnd);
    putPointsOfLineToCloud(pointCloud_msg, lineEquationC, lineCmStart, lineCmEnd);
}
lineEquation computeLineSlope(pointCm lineCmStart, pointCm lineCmEnd)///Done
{
    lineEquation lineEquationC;
    if (lineCmEnd.x == lineCmStart.x)
    {
        ///perpendicular line
        lineEquationC.slope = 0;
        lineEquationC.yIntercept = 0;
    }
    else
    {
        lineEquationC.slope = (double)(lineCmEnd.y - lineCmStart.y)/(lineCmEnd.x - lineCmStart.x);
        lineEquationC.yIntercept = lineCmStart.y - (double)(lineEquationC.slope*lineCmStart.x);
    }
    return lineEquationC;
}
void putPointsOfLineToCloud(sensor_msgs::PointCloud *pointCloud_msg, lineEquation lineEquationC, pointCm lineCmStart, pointCm lineCmEnd)///Done
{
    //point cloud
    geometry_msgs::Point32 pavPoint;
    pavPoint.x = lineCmStart.x;
    pavPoint.y = lineCmStart.y;
    pavPoint.z = DEFAULT_PAV_Z;
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
                pointCloud_msg->points.push_back(pavPoint);
                if (pointCloud_msg->points.size() > MAX_NUM_POINTS_POINTCLOUD)
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
                pointCloud_msg->points.push_back(pavPoint);
                if (pointCloud_msg->points.size() > MAX_NUM_POINTS_POINTCLOUD)
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
double cm2m(double cm)
{
    return cm/100.0;
}

double m2cm(double m)
{
    return m*100.0;
}

//v buducnosti sa iba zadaju kalibracne udaje sirka v dvoch vyskach obrazu toho isteho objektu
lineEquation calibrationWidth()
{
    lineEquation lineEquation1;
    return  lineEquation1;
}
//v buducnosit sa iba zadaju kalibracne udaje dlzka doho isteho objektu v dvoch poziciach
void calibrationDistance()
{

}
