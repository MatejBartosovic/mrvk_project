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
#define OVEREXPOSED 245
#define UNDEREXPOSED 30
#include "SidewalkEdge.h"

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
cluster123 clust_sample_main, clust_sample_buffer;
int bufferCnt;
bool first_time = true;


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

/**
 *Performs C1C2C3 color space image conversion, inRange segmenation, morphological ops (errode/dillate),
 *masks under/over exposed pixels,  scans an image region and 
 *finds object contours, reduces small contours (impurities)
 *TO DO: scitavanie framov- Kalman??, 
*/

cluster123::cluster123(){
	point = (cv::Vec3d(0.0,0.0,0.0));
	covar = (Mat::zeros(3, 3, CV_64F));
	mass = (0);
}

//Convert to c1c2c3 color space.
cv::Mat convertc123(cv::Mat in_imageRGB)
{
	cv::Mat image123;
	image123.create(in_imageRGB.rows,in_imageRGB.cols, CV_64FC3);
	double max;
	int i,j;
	for (i=0; i< in_imageRGB.rows;i++)
	{
		for (j=0;j<in_imageRGB.cols;j++)
		{
			max = std::max((double)in_imageRGB.at<cv::Vec3b>(i,j)[1]*1.0,(double)in_imageRGB.at<cv::Vec3b>(i,j)[2]);
			image123.at<cv::Vec3d>(i,j)[0] = atan((in_imageRGB.at<cv::Vec3b>(i,j)[0])/(max)) ;
			max = std::max((double)in_imageRGB.at<cv::Vec3b>(i,j)[2]*1.0,(double)in_imageRGB.at<cv::Vec3b>(i,j)[0]);
			image123.at<cv::Vec3d>(i,j)[1] = atan((in_imageRGB.at<cv::Vec3b>(i,j)[1])/(max));
			max = std::max((double)in_imageRGB.at<cv::Vec3b>(i,j)[0]*1.0,(double)in_imageRGB.at<cv::Vec3b>(i,j)[1]);
			image123.at<cv::Vec3d>(i,j)[2] = atan((in_imageRGB.at<cv::Vec3b>(i,j)[2])/(max));
			//std::cout << "Sample atan>> "<< atan ((in_imageRGB.at<cv::Vec3b>(i,j)[2]/(max))) <<" Converting RGB: "<< in_imageRGB.at<cv::Vec3b>(i,j)<< " ";
			//std::cout << " Converted c1c2c3 is: "<< image123.at<cv::Vec3d>(i,j) << std::endl;
			//cv::waitKey();
		}
	}

	return image123;

}

double setRange123(double value, double range, bool flag)
{
	//if (isHue)
	//	{
	//		if ((value < 100)&&((value+range>70))) value = 175;
	//		return value;
	//	}
	value+=range;
	return value;
}

cv::Mat maskExposure123(cv::Mat unmasked_image, cv::Mat unmasked_imageRGB)
{
	cv::Mat masked_image=unmasked_imageRGB;

	int i,j;
	for (i=0; i< unmasked_image.rows;i++)
	{
		for (j=0;j< unmasked_image.cols;j++)
		{
			if (unmasked_image.at<cv::Vec3b>(i,j)[2] > OVEREXPOSED )
			{
				masked_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(1,1,1);
			} else if (unmasked_image.at<cv::Vec3b>(i,j)[2] < UNDEREXPOSED)
			{
				masked_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(1,1,1);
			}
		}
	}
	return masked_image;
}

bool improveShadows(cv::Mat iputImgHSV,cv::Mat inputImgRGB)
{
	//volat az po maskovani
	int i,j;
	for (i=0;i<inputImgRGB.rows;i++)
	{
		for (j=0;j<inputImgRGB.cols;j++)
		{
			if ((iputImgHSV.at<cv::Vec3b>(i,j)[2] < UNDEREXPOSED+30)&&(inputImgRGB.at<cv::Vec3b>(i,j)[0]!=1)&&(inputImgRGB.at<cv::Vec3b>(i,j)[1]!=1))
			{
				inputImgRGB.at<cv::Vec3b>(i,j)[0]+=60;
				inputImgRGB.at<cv::Vec3b>(i,j)[1]+=60;
				inputImgRGB.at<cv::Vec3b>(i,j)[2]+=60;

			} else if ((iputImgHSV.at<cv::Vec3b>(i,j)[2] > OVEREXPOSED-30)&&(inputImgRGB.at<cv::Vec3b>(i,j)[0]!=1)&&(inputImgRGB.at<cv::Vec3b>(i,j)[1]!=1))
			{
				inputImgRGB.at<cv::Vec3b>(i,j)[0]-=40;
				inputImgRGB.at<cv::Vec3b>(i,j)[1]-=40;
				inputImgRGB.at<cv::Vec3b>(i,j)[2]-=40;
			}
		}
	}


	return 0;
}

cluster123 extractRegion123(cv::Mat unregioned_image, cv::Mat mask_image)
{
	cluster123 clust;
	cv::Vec3d sum_pix= cv::Vec3d(0.0,0.0,0.0);

	//cv::Point3d sum_pix;
	int pixel_cnt = 0;
	//double c1,c2,c3;
	int i,j,k,l;


	for (i=2*(unregioned_image.rows)/3; i < (unregioned_image.rows);i++)
	{
		for (j=unregioned_image.cols/3; j < 2*(unregioned_image.cols)/3;j++)
		{
			if (!((mask_image.at<cv::Vec3b>(i,j)[0]==1)&&
					(mask_image.at<cv::Vec3b>(i,j)[1]==1) &&
					(mask_image.at<cv::Vec3b>(i,j)[2]==1 )))
			if(true)
			{
				//std::cout << "\nNew adder 1 pix: " << unregioned_image.at<cv::Point3d>(i,j)<<"\n";
				pixel_cnt++;
				//cv::Vec3d cVal = unregioned_image.at<cv::Vec3d>(i,j);
				sum_pix += unregioned_image.at<Vec3d>(i,j);
				//std::cout << "New adder: " << unregioned_image.at<cv::Vec3d>(i,j) << " "<< pixel_cnt << " Sum of pixels in extractRegion is: " <<  sum_pix << std::endl;
				//cv::waitKey();
			}
				//fill region
			//unregioned_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(0,0,0);
		}
	}
	clust.point = sum_pix/pixel_cnt;
	clust.mass = pixel_cnt;
	cv::Mat meanMatClust = (Mat::zeros(3, 3, CV_64F));
	cv::Mat meanMatPixel = (Mat::zeros(3, 3, CV_64F));
	//cv::Mat meanMatPixel = (Mat::zeros(3, 3, CV_64F));

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			meanMatClust.at<double>(i,j)=clust.point[i]*clust.point[j];
		}
	}

	//std::cout << "Mean vector: \n" << clust.point <<" \n";
	//std::cout << "Mean matrix: \n" << meanMatClust <<" \n";


	for (i=2*(unregioned_image.rows)/3; i < (unregioned_image.rows);i++)
		{
			for (j=unregioned_image.cols/3; j < 2*(unregioned_image.cols)/3;j++)
			{
				if (!((mask_image.at<cv::Vec3b>(i,j)[0]==1)&&
						(mask_image.at<cv::Vec3b>(i,j)[1]==1) &&
						(mask_image.at<cv::Vec3b>(i,j)[2]==1 )))
				{
					for (k=0;k<3;k++)
					{
						for (l=0;l<3;l++)
						{
							meanMatPixel.at<double>(k,l) = unregioned_image.at<Vec3d>(i,j)[k] * unregioned_image.at<Vec3d>(i,j)[l];

							//std::cout << "Unregioned image at i,j: \n" << unregioned_image.at<Vec3d>(i,j) <<" \n";

						}
					}
					//std::cout << "Mean matrix pixel: \n" << meanMatPixel <<" \n";
					clust.covar += meanMatPixel - meanMatClust ;
					//std::cout << "Mean matrix clust: \n" << clust.covar <<" \n";

				}
					//fill region
				//unregioned_image.at<cv::Vec3b>(i,j)=cv::Point3_<unsigned char>(0,0,0);
			}
		}
	clust.covar/=clust.mass;
	clust.covar.at<double>(0,0) += 0.01 ;
	clust.covar.at<double>(1,1) += 0.01 ;
	clust.covar.at<double>(2,2) += 0.01 ;
	//std::cout << "Covariance matrix is: \n" << clust.covar <<" mass is>:"<< clust.mass << " \n";
	//std::cout << "extractRegion Cluster point is: "<< clust.point << " mass: " << clust.mass  << std::endl;
	return clust;
}

double calcClusterDistance123(cluster123 clustLearnt, cluster123 clustTraining)
{
	//std::cout << "Learnt mean vector:\n " << clustLearnt.point <<"\n";
	//std::cout << "Training mean vector:\n " << clustTraining.point <<"\n";

	//std::cout << "Learnt mass:\n " << clustLearnt.mass <<"\n";
	//std::cout << "Training mass:\n " << clustTraining.mass <<"\n";

	//std::cout << "Learnt covariance:\n " << clustLearnt.covar <<"\n";
	//std::cout << "Training covariance:\n " << clustTraining.covar <<"\n";


	cv::Mat matInvCovar = clustLearnt.covar.clone();
	matInvCovar +=  clustTraining.covar;

	cv::Vec3d meanDifferen = clustLearnt.point;
	meanDifferen -= clustTraining.point;

	Vec3d outDiff;
	double outputValue;

	//std::cout << "Covar L+T :\n " << matInvCovar<<"\n";


	cv::invert(matInvCovar , matInvCovar );
	//std::cout << "Inverted covar L+T :\n " << matInvCovar<<"\n";
	//std::cout << "MeanDifference :\n " << meanDifferen <<"\n";
	int i,j;

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			outDiff[i] += meanDifferen[j]*matInvCovar.at<double>(j,i);
			//cout << "Sucin meanDiff a invCovar \n" << outDiff<< "\n";

		}
	}

	for (i=0;i<3;i++)
		{
			outputValue += outDiff[i]*meanDifferen[i];
		}
	cout << "Calculated cluster difference is: " << outputValue << "\n";
	return outputValue ;
}

double calcPointDistance123(cluster123 clustLearnt, cluster123 clustTraining)
{
	return ((clustTraining.point[0]-clustLearnt.point[0])+
			(clustTraining.point[1]-clustLearnt.point[1])+
			(clustTraining.point[1]-clustLearnt.point[2])
	)/3.0;
}


cluster123 updateModel123(cluster123 clustLearnt, cluster123 clustTraining)
{
	//std::cout << "Mass L, mass T: " << clustLearnt.mass << " " << clustTraining.mass << "\n";
	//std::cout << "Mean L, Mean T: \n" << clustLearnt.point << "\n" << clustTraining.point << "\n";
	cluster123 clustUpdated;
	clustUpdated.point = ((clustLearnt.mass*clustLearnt.point )+ (clustTraining.mass*clustTraining.point))/(clustLearnt.mass +clustTraining.mass);
	clustUpdated.mass = (clustLearnt.mass + clustTraining.mass)/2;
	clustUpdated.covar = ((clustLearnt.mass*clustLearnt.covar) + (clustTraining.mass*clustTraining.covar))/(clustLearnt.mass + clustTraining.mass);
	//std::cout << "Updated point: " << clustUpdated.point << "\n";
	//std::cout << "Updated mass: " << clustUpdated.mass<< "\n";
	//std::cout << "Updated covar: " << clustUpdated.covar << "\n";

	return clustUpdated;
}



cv::Mat picture_segmentation_frame_c1c2c3(cv::Mat frame)
{
	cv::Mat imageHSV;		//Create Matrix to store processed image
	cv::Mat imageCont;
	cv::Mat imageThresh;
	Mat image123 = convertc123(frame);

	cluster123 clust_sample;
	cv::cvtColor(frame,imageHSV,CV_BGR2HSV);
	//cv::blur(image,imageThresh,cv::Size(60, 60));//blurr image
	frame = maskExposure123(imageHSV,frame);	
	

	double rangeH123rangeC1 = 0.05; // 0.08 -> more tolerant
	double rangeH123rangeC2 = 0.05; // 0.08 -> more tolerant
	double rangeH123rangeC3 = 0.15;
	//std::cout<<regionX<<" x "<<regionY<<" Data: "<< imHSV.at<cv::Vec3b>(regionX,regionY)<<endl;

	//region selection
	//region selection
	if (first_time)
		{
		first_time = false;
		clust_sample_main= extractRegion123 (image123,frame);
		clust_sample = clust_sample_main;
		}
	else
	{
		clust_sample= extractRegion123 (image123,frame);
	}

	double clusterDistance = calcClusterDistance123(clust_sample_main,clust_sample);
	double clusterPointDistance = calcPointDistance123(clust_sample_main,clust_sample);

	if (std::abs(clusterPointDistance)> 0.015)
	{
		//clust_sample = clust_sample_main;
		std::cout << "                      ---------- ZLY MODEL ------------\n";
	}
	else clust_sample_main = clust_sample; // - Nahrad jemnym updatom nie replacom
	
	double iLowC1 = setRange123(clust_sample.point[0],-rangeH123rangeC1,true);
	double iHighC1 = setRange123(clust_sample.point[0],+rangeH123rangeC1,true);
	double iLowC2 = setRange123(clust_sample.point[1],-rangeH123rangeC2,false);
	double iHighC2 = setRange123(clust_sample.point[1],+rangeH123rangeC2,false);
	double iLowC3 = setRange123(clust_sample.point[2],-rangeH123rangeC3,false);
	double iHighC3 = setRange123(clust_sample.point[2],+rangeH123rangeC3,false);
	

	//cout<< "ilowH, iHighH, iLowS,iHighS:"<<iLowC1<<" "<<iHighC1<<" "<< iLowC2<<" "<<iHighC2<<" "<< iLowC3 <<" "<< iHighC3<<endl;
	inRange(image123, Scalar(iLowC1, iLowC2, iLowC3), Scalar(iHighC1, iHighC2, iHighC3), imageThresh); //Threshold the image
		
	// Add Adams sidewalk lines
	cv::Point3_<unsigned char> maskedPix = cv::Vec3b(1,1,1);
	int i,j;
	for (i=0; i< frame.rows;i++)
		{
			for (j=0;j< frame.cols;j++)
			{
				if (frame.at<cv::Vec3b>(i,j) == (cv::Vec3b)maskedPix  )
				{
					imageThresh.at<uchar>(i,j,0)=100;
				}
			}
		}



	//morphological opening (remove small objects from the foreground)
	erode(imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(erode_size, erode_size)) );
	dilate( imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size, dilate_size)) );
	
	//morphological closing (fill small holes in the foreground)
	dilate( imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size*5, dilate_size*5)) );
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
	/*for( int i = 0; i< contours.size(); i++ )
	{
		if (radius[i]>180)
		{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( imageContFiltered, contours, i, color, 2, 8, hierarchy, 0, Point() );
			 floodFill(imageContFiltered, center[i], Scalar(255)); // unsafe function - please detect hranice chodnika podla krajnych kontur, nie color fill
		}
	}
	*/
	  for (i=0; i< imageThresh.rows;i++)
	  		{
	  			for (j=0;j< imageThresh.cols;j++)
	  			{
	  				if (imageThresh.at<uchar>(i,j) > 1  )
	  				{
	  					imageContFiltered.at<cv::Vec3b>(i,j,0)=255;
	  					//im_contFin.at<uchar>(i,j,1)=255;
	  					//im_contFin.at<uchar>(i,j,2)=255;
	  				}
	  			}
	  		}
		

    return imageContFiltered;
}


cv::Mat picture_segmentation_frame_c1c2c3_check(cv::Mat frame, short *valid,  SidewalkEdges *sidewalkEdges)
{
	cv::Mat imageHSV;		//Create Matrix to store processed image
	cv::Mat imageCont;
	cv::Mat imageThresh;
	Mat image123 = convertc123(frame);

	cluster123 clust_sample;
	cv::cvtColor(frame,imageHSV,CV_BGR2HSV);
	//cv::blur(image,imageThresh,cv::Size(60, 60));//blurr image
	frame = maskExposure123(imageHSV,frame);	
	improveShadows(imageHSV,frame);	

	double rangeH123rangeC1 = 0.07; // 0.08 -> more tolerant
	double rangeH123rangeC2 = 0.07; // 0.08 -> more tolerant
	double rangeH123rangeC3 = 0.15;
	//std::cout<<regionX<<" x "<<regionY<<" Data: "<< imHSV.at<cv::Vec3b>(regionX,regionY)<<endl;

	//region selection
	//region selection
	if (first_time)
		{
		first_time = false;
		clust_sample_main= extractRegion123 (image123,frame);
		clust_sample = clust_sample_main;
		clust_sample_buffer = clust_sample;
		}
	else
	{
		clust_sample= extractRegion123 (image123,frame);
	}

	double bufferDistance = calcClusterDistance123(clust_sample_buffer, clust_sample);
	
	if (abs(bufferDistance)<0.03)
	{
		bufferCnt++;
		std::cout<< "cnt++ je: " << bufferCnt<<"\n";
		if (bufferCnt>=5)
		{
			clust_sample_main = clust_sample_buffer;
			bufferCnt = 0;
		}
	} else bufferCnt=0;
	clust_sample_buffer = clust_sample;
	
	double clusterDistance = calcClusterDistance123(clust_sample_main,clust_sample);
	double clusterPointDistance = calcPointDistance123(clust_sample_main,clust_sample);

	if ((std::abs(clusterDistance)< 0.05)&& !((sidewalkEdges->right.invalidFrame)||(sidewalkEdges->left.invalidFrame)))
	{	updateModel123(clust_sample_main,clust_sample);
	 	*valid = 0;

	} else
	{
		clust_sample = clust_sample_main;
		*valid = -1;
		std::cout << "                      ---------- ZLY MODEL ------------\n";
	}
	
	double iLowC1 = setRange123(clust_sample.point[0],-rangeH123rangeC1,true);
	double iHighC1 = setRange123(clust_sample.point[0],+rangeH123rangeC1,true);
	double iLowC2 = setRange123(clust_sample.point[1],-rangeH123rangeC2,false);
	double iHighC2 = setRange123(clust_sample.point[1],+rangeH123rangeC2,false);
	double iLowC3 = setRange123(clust_sample.point[2],-rangeH123rangeC3,false);
	double iHighC3 = setRange123(clust_sample.point[2],+rangeH123rangeC3,false);
	

	//cout<< "ilowH, iHighH, iLowS,iHighS:"<<iLowC1<<" "<<iHighC1<<" "<< iLowC2<<" "<<iHighC2<<" "<< iLowC3 <<" "<< iHighC3<<endl;
	inRange(image123, Scalar(iLowC1, iLowC2, iLowC3), Scalar(iHighC1, iHighC2, iHighC3), imageThresh); //Threshold the image
		
	// Add Adams sidewalk lines
	cv::Point3_<unsigned char> maskedPix = cv::Vec3b(1,1,1);
	int i,j;
	for (i=0; i< frame.rows;i++)
		{
			for (j=0;j< frame.cols;j++)
			{
				if (frame.at<cv::Vec3b>(i,j) == (cv::Vec3b)maskedPix  )
				{
					imageThresh.at<uchar>(i,j,0)=100;
				}
			}
		}



	//morphological opening (remove small objects from the foreground)
	erode(imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(erode_size, erode_size)) );
	dilate( imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size, dilate_size)) );
	
	//morphological closing (fill small holes in the foreground)
	dilate( imageThresh, imageThresh, getStructuringElement(MORPH_ELLIPSE, Size(dilate_size*5, dilate_size*5)) );
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
	/*for( int i = 0; i< contours.size(); i++ )
	{
		if (radius[i]>180)
		{
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( imageContFiltered, contours, i, color, 2, 8, hierarchy, 0, Point() );
			 floodFill(imageContFiltered, center[i], Scalar(255)); // unsafe function - please detect hranice chodnika podla krajnych kontur, nie color fill
		}
	}
	*/
	  for (i=0; i< imageThresh.rows;i++)
	  		{
	  			for (j=0;j< imageThresh.cols;j++)
	  			{
	  				if (imageThresh.at<uchar>(i,j) > 1  )
	  				{
	  					imageContFiltered.at<cv::Vec3b>(i,j,0)=255;
	  					//im_contFin.at<uchar>(i,j,1)=255;
	  					//im_contFin.at<uchar>(i,j,2)=255;
	  				}
	  			}
	  		}
		

    return imageContFiltered;
}
