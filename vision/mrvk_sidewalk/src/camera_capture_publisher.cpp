//
// Created by smadas on 24.7.2018.
//

#include "ros/ros.h"
#include "sensor_msgs/Image.h"

//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <opencv2/objdetect/objdetect.hpp>

#include <sstream>


using namespace cv;
using namespace std;
int main(int argc, char **argv)
{

    ros::init(argc, argv, "camera_capture_publisher");

    ros::NodeHandle n;
    ros::Publisher camera_image_pub = n.advertise<sensor_msgs::Image>("camera_image", 1);
    ros::Rate loop_rate(1);

    //init camera
    CvCapture* capture = 0;
    Mat frame, frameCopy, image;
    capture = cvCaptureFromCAM( 0 ); //0=default, -1=any camera, 1..99=your camera
    if(!capture) cout << "No camera detected" << endl;
    cvNamedWindow( "result", 1 );

    int count = 0;
    while (ros::ok())
    {
//        sensor_msgs::ImageConstPtr img_msg;
//
//        //capture
//        if( capture ) {
//            cout << "In capture ..." << endl;
//            for (;;) {
//                IplImage *iplImg = cvQueryFrame(capture);
//                /*frame = iplImg;
//                if (frame.empty())
//                    break;
//                if (iplImg->origin == IPL_ORIGIN_TL)
//                    frame.copyTo(frameCopy);
//                else
//                    flip(frame, frameCopy, 0);*/
//            }
//        }
//        camera_image_pub.publish(img_msg);
//
//        ros::spinOnce();
//
//        loop_rate.sleep();
//        ++count;
    }


    //deinit camera
//    if (waitKey(10) >= 0)
//        cvReleaseCapture(&capture);
//    waitKey(0);
//    cvDestroyWindow("result");

    return 0;
}