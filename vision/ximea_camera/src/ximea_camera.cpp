#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include "xiApiPlusOcv.hpp"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <iostream>
#include <string>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>
using namespace cv;
using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	// Sample for XIMEA OpenCV
	xiAPIplusCameraOcv cam;
	// Retrieving a handle to the camera device
	printf("Opening first camera...\n");
	cam.OpenFirst();
	 
		//Set exposure
		cam.SetExposureTime(18000); //10000 us = 10 ms
		//cam.SetDownsamplingType(XI_DOWNSAMPLING_TYPE::XI_SKIPPING);
		//cam.SetDownsampling(XI_DOWNSAMPLING_VALUE::XI_DWN_4x4);
		// Note: The default parameters of each camera might be different in different API versions
		
		cam.SetWhiteBalanceRed(1.4795f);
		cam.SetWhiteBalanceGreen(1.0f);
		cam.SetWhiteBalanceBlue(1.21948f);
		cam.SetImageDataFormat(XI_RGB24);
		cam.SetDeviceOutputDataBitDepth(XI_BPP_8);
		//cam.SetImageDataBitDepth(XI_BIT_DEPTH::XI_BPP_8);
		//cam.SetSensorDataBitDepth(XI_BIT_DEPTH::XI_BPP_8);
		//cam.SetRegion_mode(1);
		cam.SetWidth(1508); // increment 4
		cam.SetHeight(850); // increment 4
		cam.SetOffsetX(252);
		cam.SetOffsetY(560);
		std::cout<<cam.GetOffsetX_Increment();
		cam.SetTriggerSource(XI_TRG_SOFTWARE);
		
		std::cout<<("\n\n Startingrting acquisition...\n");
		cam.StartAcquisition();
		std::cout<<("\n\n Triggering first...\n");
		
  		cam.SetTriggerSoftware(1);
		Mat cv_mat_image = cam.GetNextImageOcvMat();
		std::cout<<("\n\n First read...\n");
		
		//cv::imshow("Image from camera",cv_mat_image);
		
		//cv::imshow("Image from camera",cv_mat_image);
		//cam.SetAutoExposureAutoGainROIoffsetX(int AutoExposureAutoGainROIoffsetX);
		//cam.SetAutoExposureAutoGainROIoffsetY(int AutoExposureAutoGainROIoffsetX);
		//cam.SetAutoExposureAutoGainROIWidth(int AutoExposureAutoGainROIWidth);
		//cam.SetAutoExposureAutoGainROIHeight(int AutoExposureAutoGainROIWidth);
		//cam.EnableAutoExposureAutoGain(); / if too bright, too dark
		//cam.DisableAutoExposureAutoGain();
		//cv::waitKey();
		//cam.SetHDRTimeSlope1();
		//cam.SetHDRTimeSlope2();
		//cam.SetHDRKnee1Percent();
		//cam.SetHDRKnee2Percent();
		//cam.EnableHDR();
		//printf("First pixel value \n");
		//#define EXPECTED_IMAGES 300
		//std::string cesta("/home/pyc/Pictures/img");
		//std::string append(".png");
		//std::string fullpath;
		//std::stringstream ss;
					




  ros::init(argc, argv, "ximea_publisher");

  ros::NodeHandle n;

  ros::Publisher ximea_img = n.advertise<sensor_msgs::Image>("ximea_img", 1);
  	

  ros::Rate loop_rate(4);
	cv_bridge::CvImage ximea_img_bridge;
	sensor_msgs::Image ximea_img_msg;
	std_msgs::Header header;
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
	std::cout<<("\n\n Starting loop...\n");
		
  while (ros::ok())
  {
	  
	  	cam.SetTriggerSoftware(1);
		cv_mat_image = cam.GetNextImageOcvMat();
	   	header.stamp = ros::Time::now();
		//cv::imshow("Image from camera",cv_mat_image);
		std::stringstream ss;
		//ss<< images;
		//fullpath = cesta + ss.str() + append;
		//cout<< fullpath <<"e\n";
		 //cvtColor(cv_mat_image, bwsrc, cv::COLOR_RGB2GRAY);
		//cv::imwrite( fullpath, cv_mat_image );
		//cvWaitKey();
		//printf("\t%d\n",bwsrc.at<unsigned char>(0,0));
		

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
  
    ROS_INFO("%s", "posielam obr \n");

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
	  
	 //publish processed image
    header.stamp = ros::Time::now();
    ximea_img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, cv_mat_image);
    ximea_img_bridge.toImageMsg(ximea_img_msg);
    ximea_img.publish(ximea_img_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
		cam.StopAcquisition();
 		//cout << cam.GetHDR_KNEEPOINT_COUNT();
		cam.Close();
		printf("Done\n");


  return 0;
}


