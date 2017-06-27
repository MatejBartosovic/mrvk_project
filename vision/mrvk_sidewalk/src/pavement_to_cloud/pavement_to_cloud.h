#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/PointCloud.h>

//Cm - centimeters

#define DEFAULT_PAV_Z 0.01 //10cm
#define PAV_LINE_RESOLUTION 0.1 //5cm
#define MAX_DISTANCE_CM 500000 //500m
#define MAX_NUM_POINTS_POINTCLOUD 10000

#define WIDTH_K -0.1//0.3051948052
#define WIDTH_Q 10//240.7532467532
#define DIST_K 1.7321
#define DIST_Q -127.3839
#define PIC_WIDTH_PIX 1280
#define PIC_HEIGHT_PIX 700
#define PIC_CALIB_WIDTH_PIX 2592
#define PIC_CALIB_HEIGHT_PIX 1944
#define CALIB_OBJ_WIDTH_MM 300//mm
#define CAMERA_POSE_X 0
#define CAMERA_POSE_Y 800 //mm
#define CAMERA_ANGLE 30 //old value 60//degrees
#define IMG_OFFSET 1000 //550 //mm

struct pointCm
{
    int x = 0;
    int y = 0;
    int z = DEFAULT_PAV_Z;
};
struct lineEquation
{
    double slope = 0;
    double yIntercept = 0;
};
struct linePointsPix
{
    cv::Point start;
    cv::Point end;
};
struct linePointsCm
{
    pointCm start;
    pointCm end;
};
struct pavFragmentPix
{
    linePointsPix left;
    linePointsPix right;
};
struct pavFragmentCm
{
    linePointsCm left;
    linePointsCm right;
};
struct pavFragment
{
    pavFragmentPix pix;
    pavFragmentCm cm;
};

void putPavementFragmentIntoCloud(sensor_msgs::PointCloud *pointCloud_msg, pavFragment *pavFragmentC);
void changePavPointsToCm(pavFragment *pavFragmentC);
int getWidthPix(cv::Point pointLeft, cv::Point pointRight);
int getWidthCm(int widthPix, int heightPix);
cv::Point getCenterOfPavement(cv::Point pointLeft, cv::Point pointRight);
pointCm getCenterOfPavementCm(int pavCenterFromImgCenterCm, cv::Point pavementCenterPix);
int computeCmX(pointCm pavCenterCm, int pavementWidthCm, bool isLeft);
void changeToCmX(pavFragment *pavFragmentC);
int getFragmentLengthPixY(linePointsCm linePointsCmC);
int getFragmentLengthCmY(int startLineY, int endLineY, pointCm *startLine, pointCm *endLine);
void changeToCmY(pavFragment *pavFragmentC);
void putPavementLineIntoCloud(sensor_msgs::PointCloud *pointCloud_msg, pointCm lineCmStart, pointCm lineCmEnd);
lineEquation computeLineSlope(pointCm lineCmStart, pointCm lineCmEnd);
void putPointsOfLineToCloud(sensor_msgs::PointCloud *pointCloud_msg, lineEquation lineEquationC, pointCm lineCmStart, pointCm lineCmEnd);


lineEquation calibrationWidth();
void calibrationDistance();