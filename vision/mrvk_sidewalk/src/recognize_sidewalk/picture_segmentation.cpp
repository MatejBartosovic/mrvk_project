//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include "segment_class.hpp"
#define dilate_size 10
#define erode_size 10
#define OVEREXPOSED 220
#define UNDEREXPOSED 30

using namespace cv;
using namespace std;

// HSV segmentation
// ok values  0 175 1 82 36 255 - TODO: update adaptation
int iLowH = 75;
int iHighH = 130;
int iLowS = 0;
int iHighS = 162;
int iLowV = 36;
int iHighV = 255;


cv::Mat picture_segmentation_frame(cv::Mat frame)
{
    cv::Mat segmented;
    segmented = frame.clone();

    cv::blur(frame,segmented,cv::Size(60, 60));//blurr image
    //remove red and blue
    for(int i = 0; i < segmented.size().width; i++)
    {
        for(int j = 0; j < segmented.size().height; j++)
        {
            //segmented.at<cv::Vec3b>(j,i)[0]= 0;
            segmented.at<cv::Vec3b>(j,i)[1]= 0;
            segmented.at<cv::Vec3b>(j,i)[2]= 0;
            //increase contrast
            if (segmented.at<cv::Vec3b>(j,i)[0] > 120)
            {
                segmented.at<cv::Vec3b>(j,i)[0] = 255;
            }
            else
            {
                segmented.at<cv::Vec3b>(j,i)[0] = 0;
            }
        }
    }
    return segmented;
}

/**
 *Performs HSV image conversion, inRange segmenation, morphological ops (errode/dillate),
 *finds object contours, reduces small contours (impurities)
 *TO DO: kalibracia farieb/svetla, kontury-DoneOK, scitavanie framov- Kalman??, adaptacia na svetlo
*/

cv::Vec3b computeAdaptationKernels(Mat imageHSV)
{
	int regionX = imageHSV.cols/2;
	int regionY = (imageHSV.rows/4)*3;

	int k = 0, i = 0, j = 0;
	int minKern=500, indexKern=0;
	cv::Vec3i kernelAvg[3];
	for (k=-1;k<=1;k++)
	{
		for (i=-1;i<=1;i++)
		{
			for (j=-1;j<=1;j++)
			{
				kernelAvg[k+1]+=imageHSV.at<cv::Vec3b>((70*k)+regionX+(i*3),+regionY+(j*3));
			}

		}
	kernelAvg[k+1]/=9;
	if (minKern>kernelAvg[k+1][0])
	{
		minKern=kernelAvg[k+1][0];
		indexKern = k+1;
	}
	}
	// Check validity
	if(abs(((iLowH+iHighH)/2-kernelAvg[indexKern][0]))>30)
		kernelAvg[indexKern][0]=(iLowH+iHighH)/2;

	cv::Vec3b rangeSamplePoint= kernelAvg[indexKern];
	cout << "kernel avg: "<< rangeSamplePoint << endl;
	return rangeSamplePoint;
}

int setRange(int value, int range)
{
	value+=range;
	if (value<0) value = 0;
	if (value>255) value = 255;
	return value;
}

cluster::cluster(){
	point = (cv::Vec3b(0,0,0));
	covar = (Mat::zeros(3, 3, CV_32S));
	mass = (0);
}
cv::Mat maskExposure(cv::Mat unmasked_image)
{
	cv::Mat masked_image=unmasked_image;

	int i,j;
	for (i=0; i< masked_image.rows;i++)
	{
		for (j=0;j<masked_image.cols;j++)
		{
			if (masked_image.at<cv::Vec3b>(i,j)[2] > OVEREXPOSED )
			{
				masked_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(0,0,0);
			} else if (masked_image.at<cv::Vec3b>(i,j)[2] < UNDEREXPOSED)
			{
				masked_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(0,0,0);
			}
		}
	}
	return masked_image;
}

cluster extractRegion(cv::Mat unregioned_image)
{
	cluster clust;
	cv::Vec3i sum_pix;
	int pixel_cnt = 0;
	int i,j;
		for (i=unregioned_image.rows; i > 2*(unregioned_image.rows)/3;i--)
		{
			for (j=unregioned_image.cols/3;j<2*(unregioned_image.cols)/3;j++)
			{
				if ((unregioned_image.at<cv::Vec3b>(i,j))==((unregioned_image.at<cv::Vec3b>(i,j))))
				{
					pixel_cnt++;
					sum_pix += unregioned_image.at<cv::Vec3b>(i,j);
					//std::cout << pixel_cnt << " Sum of pixels in extractRegion is: " <<  sum_pix << std::endl;
				}

				//fill region
				//unregioned_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(0,0,0);
			}
		}
	clust.point = sum_pix/pixel_cnt;
	clust.mass = pixel_cnt;
	//std::cout << "extractRegion Cluster point is: "<< clust.point << " mass: " << clust.mass  << std::endl;
	return clust;
}


cv::Mat picture_segmentation_frame_HSV(cv::Mat frame)
{
	cv::Mat imageHSV;		//Create Matrix to store processed image
	cv::Mat imageCont;
	cv::Mat imageThresh;

	cluster clust_sample;
	cv::cvtColor(frame,imageHSV,CV_BGR2HSV);
	//cv::blur(image,imageThresh,cv::Size(60, 60));//blurr image
	imageHSV = maskExposure(imageHSV);	
	
	//region selection
	clust_sample= extractRegion (imageHSV);

	int rangeH = 35;
	int rangeS = 70;

	iLowH = setRange(clust_sample.point[0],-rangeH);
	iHighH = setRange(clust_sample.point[0],+rangeH);
	iLowS = setRange(clust_sample.point[1],-rangeS);
	iHighS = setRange(clust_sample.point[1],+rangeS);

	inRange(imageHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imageThresh); //Threshold the image	
		
	//morphological opening (remove small objects from the foreground)
	erode(imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(erode_size, erode_size)) );
	dilate( imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size, dilate_size)) );
	
	//morphological closing (fill small holes in the foreground)
	dilate( imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size*3, dilate_size*3)) );
	erode(imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(erode_size, erode_size)) );
	
	//contours
 	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;
  	RNG rng(12345);
  	imageCont=imageThresh.clone();
	findContours( imageCont, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		
		
	// Approximate contours to polygons + get bounding circles
    vector<vector<Point> > contours_poly( contours.size() );
	vector<Point2f> center( contours.size() );
	vector<float> radius( contours.size() );  
    
	for( int i = 0; i < contours.size(); i++ )
    { 
        approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
    }

	/// Draw contours*/
	  Mat imageContFiltered = Mat::zeros( imageCont.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	{
		if (radius[i]>180)
		{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( imageContFiltered, contours, i, color, 2, 8, hierarchy, 0, Point() );
			 floodFill(imageContFiltered, center[i], Scalar(255)); // unsafe function - please detect hranice chodnika podla krajnych kontur, nie color fill
		}
	}
		

    return imageContFiltered;
}
