//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

#define dilate_size 10
#define erode_size 10

using namespace cv;

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
cv::Mat picture_segmentation_frame_HSV(cv::Mat frame)
{
	Mat imageHSV;		//Create Matrix to store processed image
	Mat imageCont;
	Mat imageThresh;

	cvtColor(frame,imageHSV,CV_BGR2HSV);
	//cv::blur(image,imageThresh,cv::Size(60, 60));//blurr image
		
	// HSV segmentation
	// ok values  0 175 1 82 36 255 - TODO: update adaptation
	int iLowH = 75;
	int iHighH = 130;
	int iLowS = 0;
	int iHighS = 162;
	int iLowV = 36;
	int iHighV = 255;

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
