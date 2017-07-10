//opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

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