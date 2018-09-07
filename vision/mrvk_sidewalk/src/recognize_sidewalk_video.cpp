//ros
#include <ros/ros.h>

//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

//octomap - pavement
#include <nav_msgs/OccupancyGrid.h>

//pointcloud - pavement
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>

//looking for home directory
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>

//file libraries
#include <iostream>
#include <fstream>

//local libraries
#include "pavement_to_marker/pavement_to_marker.h"
#include "pavement_to_cloud/pavement_to_cloud.h"
#include "misc_tools/misc_tools.h"
#include "recognize_sidewalk/recognize_sidewalk.h"

#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

//v4l2 libs
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#define EDGE_MARKER_WIDTH 6
#define EDGE_MARKER_TYPE 8
#define EDGE_MARKER_SHIFT 0

#define EDGE_PAV_VERTICAL_START 150
#define EDGE_PAV_VERTICAL_END 650
#define EDGE_MARKER_VERTICAL_POINT_DIST 50

using namespace cv;

uint8_t *buffer;

static int xioctl(int fd, int request, void *arg)
{
    int r;

    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}

int print_caps(int fd)
{
    struct v4l2_capability caps = {};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps))
    {
        perror("Querying Capabilities");
        return 1;
    }

    printf( "Driver Caps:\n"
                    "  Driver: \"%s\"\n"
                    "  Card: \"%s\"\n"
                    "  Bus: \"%s\"\n"
                    "  Version: %d.%d\n"
                    "  Capabilities: %08x\n",
            caps.driver,
            caps.card,
            caps.bus_info,
            (caps.version>>16)&&0xff,
            (caps.version>>24)&&0xff,
            caps.capabilities);

    ROS_ERROR("kokot1");
    struct v4l2_cropcap cropcap = {0};
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ROS_ERROR("kokot1.1");
    /*if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
    {
        perror("Querying Cropping Capabilities");
        return 1;
    }*/
    ROS_ERROR("kokot2");
    printf( "Camera Cropping:\n"
                    "  Bounds: %dx%d+%d+%d\n"
                    "  Default: %dx%d+%d+%d\n"
                    "  Aspect: %d/%d\n",
            cropcap.bounds.width, cropcap.bounds.height, cropcap.bounds.left, cropcap.bounds.top,
            cropcap.defrect.width, cropcap.defrect.height, cropcap.defrect.left, cropcap.defrect.top,
            cropcap.pixelaspect.numerator, cropcap.pixelaspect.denominator);

    int support_grbg10 = 0;
    ROS_ERROR("kokot3");
    struct v4l2_fmtdesc fmtdesc = {0};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    char fourcc[5] = {0};
    char c, e;
    printf("  FMT : CE Desc\n--------------------\n");
    while (0 == xioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc))
    {
        strncpy(fourcc, (char *)&fmtdesc.pixelformat, 4);
        if (fmtdesc.pixelformat == V4L2_PIX_FMT_SGRBG10)
            support_grbg10 = 1;
        c = fmtdesc.flags & 1? 'C' : ' ';
        e = fmtdesc.flags & 2? 'E' : ' ';
        printf("  %s: %c%c %s\n", fourcc, c, e, fmtdesc.description);
        fmtdesc.index++;
    }
    ROS_ERROR("kokot4");
    /*
    if (!support_grbg10)
    {
        printf("Doesn't support GRBG10.\n");
        return 1;
    }*/

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
    //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
    {
        perror("Setting Pixel Format");
        return 1;
    }
    ROS_ERROR("kokot5");
    strncpy(fourcc, (char *)&fmt.fmt.pix.pixelformat, 4);
    printf( "Selected Camera Mode:\n"
                    "  Width: %d\n"
                    "  Height: %d\n"
                    "  PixFmt: %s\n"
                    "  Field: %d\n",
            fmt.fmt.pix.width,
            fmt.fmt.pix.height,
            fourcc,
            fmt.fmt.pix.field);

    return 0;
}

int init_mmap(int fd)
{
    struct v4l2_requestbuffers req = {0};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req))
    {
        perror("Requesting Buffer");
        return 1;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
    {
        perror("Querying Buffer");
        return 1;
    }

    buffer = static_cast<uint8_t*>( mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset));
    printf("Length: %d\nAddress: %p\n", buf.length, buffer);
    printf("Image Length: %d\n", buf.bytesused);
    ROS_ERROR("kurvafix");
    return 0;
}

int capture_image(int fd)
{
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
    {
        perror("Query Buffer");
        return 1;
    }

    if(-1 == xioctl(fd, VIDIOC_STREAMON, &buf.type))
    {
        perror("Start Capture");
        return 1;
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    struct timeval tv = {0};
    tv.tv_sec = 2;
    int r = select(fd+1, &fds, NULL, NULL, &tv);
    if(-1 == r)
    {
        perror("Waiting for Frame");
        return 1;
    }

    if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
    {
        perror("Retrieving Frame");
        return 1;
    }
    printf ("saving image\n");

    IplImage* frame;
    CvMat cvmat = cvMat(640, 480, CV_8UC3, (void*)buffer);
    frame = cvDecodeImage(&cvmat, 1);
    cvNamedWindow("window",CV_WINDOW_AUTOSIZE);
    cvShowImage("window", frame);
    cvWaitKey(0);
    cvSaveImage("image.jpg", frame, 0);

    return 0;
}

class CvImage
{
    sensor_msgs::ImagePtr toImageMsg() const;

    void toImageMsg(sensor_msgs::Image& ros_image) const;
};

//int getLeftPavementPoint(cv::Mat image, int line);
//int getRightPavementPoint(cv::Mat image, int line);

int main(int argc, char **argv) {

    //START ros init
    ros::init(argc, argv, "recognize_sidewalk_video");
    ros::NodeHandle n;

    //START get parameters
    RecognizeSidewalkParams params;
    params.getParametersFromServer(n);
    ros::Rate loop_rate(params.spinFreq);
    //END get parameters

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg;
    sensor_msgs::Image img_msg_orig;
    std_msgs::Header header;

    //init publishers
    ros::Publisher pub_img = n.advertise<sensor_msgs::Image>("video_image_topic", 1);//output image publisher
    ros::Publisher pub_img_orig = n.advertise<sensor_msgs::Image>("video_image_orig_topic", 1);//output image publisher
    ros::Publisher octomap_pub = n.advertise<nav_msgs::OccupancyGrid>("pavement_map", 1);//map occupancy publisher
    ros::Publisher pub_pav_pointCloud = n.advertise<sensor_msgs::PointCloud> ("pav_pointCloud", 1);//publisher for pavement point cloud
    //END ros init

    //point cloud
    sensor_msgs::PointCloud pointCloud_msg;
    geometry_msgs::Point32 pavPoint;
    pointCloud_msg.header.stamp = ros::Time::now();
    pointCloud_msg.header.frame_id = "map";

    //START video directory
    std::string videoDirectory;
    videoDirectory = get_directory("/Videos/MRVKroute/", "obed", "", "mp4");
    //END video directory

    Mat image;          //Create Matrix to store image
    Mat imageResult;		//Create Matrix to store processed image
    Mat imageOrig;			//Create Matrix to store orig image with detected pavement

    VideoCapture cap;          //initialize capture
    //cap.open(1);//videoDirectory.c_str());

    //START android cam
    const char* message = "Capture a frame!";
    const char* errorMessage = "Could not open the camera!";


        // print message to console
       /** printf("%s\n", message);
        cv::VideoCapture capture(CV_CAP_ANDROID + 1);
//cv::VideoCapture capture(CV_CAP_ANDROID + 1);//front camera for Android 2.3.3 or  newer
        if( !capture.isOpened() )
        {
            printf("%s\n", errorMessage);
            return 0;
        }
        capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);**/

        /*cv::Mat frame;
        capture >> frame;
        if( !frame.empty() ){
            cv::imwrite("/mnt/sdcard/CaptureFrame.png", frame);
        }*/

    //END android cam

    //START v4l2
    int fd;

    fd = open("/dev/video0", O_RDWR);
    ROS_ERROR("kokot");
    if (fd == -1)
    {
        perror("Opening video device");
        return 1;
    }
    else
    {
        perror("fuck my life");
        perror("fuck my life");
        perror("fuck my life");
    }
    ROS_ERROR("pica");
    if(print_caps(fd))
    {
        //return 1;
    }

    ROS_ERROR("rododendron");
    if(init_mmap(fd))
        return 1;
    int i;
    ROS_ERROR("rododendron");
    for(i=0; i<5; i++)
    {
        if(capture_image(fd))
            return 1;
    }
    close(fd);
    //END v4l2

    //cap = cv::cv2.VideoCapture('http://192.168.0.21:4747/mjpegfeed');
    //START edge detection variables
    SidewalkEdges sidewalkEdges;
    Point lineStart;// = Point(100, 100);
    Point lineEnd;// = Point(300, 300);
    Point lineStartRight;// = Point(100, 100);
    Point lineEndRight;// = Point(300, 300);
    int num_of_edge_points = (EDGE_PAV_VERTICAL_END - EDGE_PAV_VERTICAL_START - EDGE_MARKER_VERTICAL_POINT_DIST)/EDGE_MARKER_VERTICAL_POINT_DIST;
    int leftPoint = 0;
    int rightPoint = 0;
    short valid_data;
    //END edge detection variables

    while (ros::ok())
    {
        //cap >> image;
        if (image.empty())
        {
            //end of video
            break;
        }
        imageOrig = image.clone();

        valid_data = recognize_sidewalk_frame(&imageOrig, &imageResult, &params, &sidewalkEdges, n);

        //publish processed image
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageResult);
        img_bridge.toImageMsg(img_msg);
        pub_img.publish(img_msg);//publish processed image
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageOrig);
        img_bridge.toImageMsg(img_msg_orig);
        pub_img_orig.publish(img_msg_orig);//publish original image
        //usleep(10000);

        //publish point cloud
        pointCloud_msg.header.stamp = ros::Time::now();
        pub_pav_pointCloud.publish(pointCloud_msg);

        //check ros events
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/*int getLeftPavementPoint(cv::Mat image, int line)
{
    int isGrass = 0;
    int isEdge = 0;
    for (int i = 0; i < image.size().width; i++)
    {
        if (image.at<cv::Vec3b>(line,i)[0] == 0)
        {
            isGrass = 1;
        }
        else if ((image.at<cv::Vec3b>(line,i)[0] == 255) && (isGrass == 1) && (isEdge == 0))
        {
            isGrass = 0;
            isEdge = 1;
            return i;
        }
    }
    return 0;
}

int getRightPavementPoint(cv::Mat image, int line)
{
    int isGrass = 0;
    int isEdge = 0;
    for (int i = image.size().width; i > 0; i--)
    {
        if (image.at<cv::Vec3b>(line,i)[0] == 0)
        {
            isGrass = 1;
        }
        else if ((image.at<cv::Vec3b>(line,i)[0] == 255) && (isGrass == 1) && (isEdge == 0))
        {
            isGrass = 0;
            isEdge = 1;
            return i;
        }
    }
    return 0;
}*/


