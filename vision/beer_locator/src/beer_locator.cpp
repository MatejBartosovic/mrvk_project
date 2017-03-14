/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

class Receiver
{
public:
  enum Mode
  {
    IMAGE = 0,
    CLOUD,
    BOTH
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth, markerPath;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

	cv::Mat img_marker;
  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed, const std::string &markerPath)
    : topicColor(topicColor), topicDepth(topicDepth), markerPath(markerPath), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
		cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

  void run(const Mode mode)
  {
    start(mode);
    stop();
  }

private:
  void start(const Mode mode)
  {
		std::cout << "Opening marker: "<< markerPath<< std::endl;
		img_marker =cv::imread(markerPath,CV_LOAD_IMAGE_GRAYSCALE);
		/*cv::imread(
					"/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/markers/Marker3.ppm",
					CV_LOAD_IMAGE_GRAYSCALE);
		*/
	  if(! img_marker.data )                              // Check for invalid input
    {
        std::cout <<  "Could not open or find the marker image" << std::endl ;
    }
		
    this->mode = mode;
    running = true;

    std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
    std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

    image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
    subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
    subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
    subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
    subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

    if(useExact)
    {
      syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }
    else
    {
      syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
      syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
    }

    spinner.start();

    std::chrono::milliseconds duration(1);
    while(!updateImage || !updateCloud)
    {
      if(!ros::ok())
      {
        return;
      }
      std::this_thread::sleep_for(duration);
    }
    cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);
    createLookup(this->color.cols, this->color.rows);

    switch(mode)
    {
    case CLOUD:
      cloudViewer();
      break;
    case IMAGE:
      imageViewer();
      break;
    case BOTH:
      imageViewerThread = std::thread(&Receiver::imageViewer, this);
      cloudViewer();
      break;
    }
  }

  void stop()
  {
    spinner.stop();

    if(useExact)
    {
      delete syncExact;
    }
    else
    {
      delete syncApproximate;
    }

    delete subImageColor;
    delete subImageDepth;
    delete subCameraInfoColor;
    delete subCameraInfoDepth;

    running = false;
    if(mode == BOTH)
    {
      imageViewerThread.join();
    }
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
    cv::Mat color, depth;

    readCameraInfo(cameraInfoColor, cameraMatrixColor);
    readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
    readImage(imageColor, color);
    readImage(imageDepth, depth);

    // IR image input
    if(color.type() == CV_16U)
    {
      cv::Mat tmp;
      color.convertTo(tmp, CV_8U, 0.02);
      cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }

    lock.lock();
    this->color = color;
    this->depth = depth;
    updateImage = true;
    updateCloud = true;
    lock.unlock();
  }

  void imageViewer()
  {
    cv::Mat color, depth, depthDisp, combined;
    std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
    double fps = 0;
    size_t frameCount = 0;
    std::ostringstream oss;
    const cv::Point pos(5, 15);
    const cv::Scalar colorText = CV_RGB(255, 255, 255);
    const double sizeText = 0.5;
    const int lineText = 1;
    const int font = cv::FONT_HERSHEY_SIMPLEX;

    cv::namedWindow("Image Viewer");
    oss << "starting...";

    start = std::chrono::high_resolution_clock::now();
    for(; running && ros::ok();)
    {
      if(updateImage)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateImage = false;
        lock.unlock();

        ++frameCount;
        now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
        if(elapsed >= 1.0)
        {
          fps = frameCount / elapsed;
          oss.str("");
          oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
          start = now;
          frameCount = 0;
        }

        dispDepth(depth, depthDisp, 12000.0f);
	detect_beer(color); // Uncomment this for detection
        combine(color, depthDisp, combined);
        //combined = color;

        cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
        cv::imshow("Image Viewer", combined);
      }

      int key = cv::waitKey(1);
      switch(key & 0xFF)
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        if(mode == IMAGE)
        {
          createCloud(depth, color, cloud);
          saveCloudAndImages(cloud, color, depth, depthDisp);
        }
        else
        {
          save = true;
        }
        break;
      }
    }
    cv::destroyAllWindows();
    cv::waitKey(100);
  }

  void cloudViewer()
  {
    cv::Mat color, depth;
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
    const std::string cloudName = "rendered";

    lock.lock();
    color = this->color;
    depth = this->depth;
    updateCloud = false;
    lock.unlock();

    createCloud(depth, color, cloud);

    visualizer->addPointCloud(cloud, cloudName);
    visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
    visualizer->initCameraParameters();
    visualizer->setBackgroundColor(0, 0, 0);
    visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
    visualizer->setSize(color.cols, color.rows);
    visualizer->setShowFPS(true);
    visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
    visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

    for(; running && ros::ok();)
    {
      if(updateCloud)
      {
        lock.lock();
        color = this->color;
        depth = this->depth;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, cloud);

        visualizer->updatePointCloud(cloud, cloudName);
      }
      if(save)
      {
        save = false;
        cv::Mat depthDisp;
        dispDepth(depth, depthDisp, 12000.0f);
        saveCloudAndImages(cloud, color, depth, depthDisp);
      }
      visualizer->spinOnce(10);
    }
    visualizer->close();
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {
    if(event.keyUp())
    {
      switch(event.getKeyCode())
      {
      case 27:
      case 'q':
        running = false;
        break;
      case ' ':
      case 's':
        save = true;
        break;
      }
    }
  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
    pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
    double *itC = cameraMatrix.ptr<double>(0, 0);
    for(size_t i = 0; i < 9; ++i, ++itC)
    {
      *itC = cameraInfo->K[i];
    }
  }

  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    #pragma omp parallel for
    for(int r = 0; r < in.rows; ++r)
    {
      const uint16_t *itI = in.ptr<uint16_t>(r);
      uint8_t *itO = tmp.ptr<uint8_t>(r);

      for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
      {
        *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
      }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
    out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

    #pragma omp parallel for
    for(int r = 0; r < inC.rows; ++r)
    {
      const cv::Vec3b
      *itC = inC.ptr<cv::Vec3b>(r),
       *itD = inD.ptr<cv::Vec3b>(r);
      cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

      for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
      {
        itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
        itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
        itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
      }
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
    oss.str("");
    oss << "./" << std::setfill('0') << std::setw(4) << frame;
    const std::string baseName = oss.str();
    const std::string cloudName = baseName + "_cloud.pcd";
    const std::string colorName = baseName + "_color.jpg";
    const std::string depthName = baseName + "_depth.png";
    const std::string depthColoredName = baseName + "_depth_colored.png";

    OUT_INFO("saving cloud: " << cloudName);
    writer.writeBinary(cloudName, *cloud);
    OUT_INFO("saving color: " << colorName);
    cv::imwrite(colorName, color, params);
    OUT_INFO("saving depth: " << depthName);
    cv::imwrite(depthName, depth, params);
    OUT_INFO("saving depth: " << depthColoredName);
    cv::imwrite(depthColoredName, depthColored, params);
    OUT_INFO("saving complete!");
    ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    const float cx = cameraMatrixColor.at<double>(0, 2);
    const float cy = cameraMatrixColor.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }

	int detect_beer(cv::Mat img_scene)
	{
	std::string scene_addr;
	std::stringstream sstm;
	cv::SurfDescriptorExtractor extractor;

	cv::Mat descriptors_object, descriptors_scene;
	struct timespec t2, t3;
	double dt1;
	int cnti=1;
	//cv::Mat img_scene;
		clock_gettime(CLOCK_MONOTONIC, &t2);
		/*sstm.str("");
		std::cout << cnti << std::endl;
		if (cnti < 10)
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_corny_hard/marker_corny_hard_0"
					<< cnti << ".png";
		else
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_corny_hard/marker_corny_hard_"
					<< cnti << ".png";

		scene_addr = sstm.str();
		std::cout << "Scene path: " << scene_addr << std::endl;
		img_scene = cv::imread(scene_addr, CV_LOAD_IMAGE_GRAYSCALE);*/

		if (!img_marker.data || !img_scene.data) {
			std::cout << " --(!) Error reading images " << std::endl;
			return -1;
		}

		int hessianThresh = 1000;
		double max_dist = 0;
		double min_dist = 80;
		cv::SurfFeatureDetector detector(hessianThresh);
		cv::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
		cv::FlannBasedMatcher matcher;
		cv::vector<cv::DMatch> matches, good_matches;
		cv::Mat img_matches, H;
		cv::vector<cv::Point2f> marker_corners(4), scene_corners(4), obj, scene;

		//Key points detection
		detector.detect(img_marker, keypoints_object);
		detector.detect(img_scene, keypoints_scene);

		//Calculate descriptors (feature vectors)
		extractor.compute(img_marker, keypoints_object, descriptors_object);
		extractor.compute(img_scene, keypoints_scene, descriptors_scene);

		// Matching descriptor vectors using FLANN matcher

		matcher.match(descriptors_object, descriptors_scene, matches);

		//-- Quick calculation of max and min distances between keypoints
		for (int i = 0; i < descriptors_object.rows; i++) {
			double dist = matches[i].distance;
			if (dist < min_dist)
				min_dist = dist;
			if (dist > max_dist)
				max_dist = dist;
		}

		std::cout << "-- Max dist :" << min_dist << std::endl;
		std::cout << "-- Max dist :" << min_dist << std::endl;

		//-- Filter good matches

		for (int i = 0; i < descriptors_object.rows; i++) {
			if (matches[i].distance < 3 * min_dist) {
				good_matches.push_back(matches[i]);
			}
		}

		img_matches = img_scene;
		cv::drawMatches(img_marker, keypoints_object, img_scene, keypoints_scene,
				good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
				cv::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		// Find the object

		for (unsigned int i = 0; i < good_matches.size(); i++) {
			//-- Get the keypoints from the good matches
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

		H = findHomography(obj, scene, CV_RANSAC);

		//-- Get the corners from the image_1 ( the object to be "detected" )

		marker_corners[0] = cvPoint(0, 0);
		marker_corners[1] = cvPoint(img_marker.cols, 0);
		marker_corners[2] = cvPoint(img_marker.cols, img_marker.rows);
		marker_corners[3] = cvPoint(0, img_marker.rows);

		perspectiveTransform(marker_corners, scene_corners, H);

		//-- Draw detected marker borders
		cv::line(img_matches, scene_corners[0]+cv::Point_<float>(img_marker.cols,0.0),	scene_corners[1] +cv::Point_<float>(img_marker.cols,0.0), cv::Scalar(0, 255, 0), 4);
		cv::line(img_matches, scene_corners[1]+cv::Point_<float>(img_marker.cols,0.0),	scene_corners[2] +cv::Point_<float>(img_marker.cols,0.0), cv::Scalar(0, 255, 0), 4);
		cv::line(img_matches, scene_corners[2]+cv::Point_<float>(img_marker.cols,0.0),	scene_corners[3] +cv::Point_<float>(img_marker.cols,0.0), cv::Scalar(0, 255, 0), 4);
		cv::line(img_matches, scene_corners[3]+cv::Point_<float>(img_marker.cols,0.0),	scene_corners[0] +cv::Point_<float>(img_marker.cols,0.0), cv::Scalar(0, 255, 0), 4);
		cv::Point2f obj_center = (scene_corners[0]+scene_corners[1]+scene_corners[2]+scene_corners[3]+cv::Point_<float>(4*img_marker.cols,0.0));
		obj_center.x/=4.0;
		obj_center.y/=4.0;
		cv::circle(img_matches, obj_center,15, cv::Scalar( 0, 0, 255 ),4,8) ;
		//-- Show detected matches
		imshow("Good Matches & Object detection", img_matches);
		clock_gettime(CLOCK_MONOTONIC, &t3);
		dt1 = (t3.tv_sec - t2.tv_sec)
				+ (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
		std::cout << "elapsed time: " << dt1 << " s  " << std::endl;
		cv::waitKey(0);
		return 0;
	}
	
 
};

void help(const std::string &path)
{
  std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
            << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl
            << FG_YELLOW "    '--marker <path_without_quotes>'" NO_COLOR " Specify marker image path" << std::endl;
}

int main(int argc, char **argv)
{
#if EXTENDED_OUTPUT
  ROSCONSOLE_AUTOINIT;
  if(!getenv("ROSCONSOLE_FORMAT"))
  {
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
  }
#endif

  ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

  if(!ros::ok())
  {
    return 0;
  }

  std::string ns = K2_DEFAULT_NS;
  std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
  std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
	std::string markerPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/markers/Marker3.ppm";
  bool useExact = true;
  bool useCompressed = false;
  Receiver::Mode mode = Receiver::CLOUD;

  for(size_t i = 1; i < (size_t)argc; ++i)
  {
    std::string param(argv[i]);

    if(param == "-h" || param == "--help" || param == "-?" || param == "--?")
    {
      help(argv[0]);
      ros::shutdown();
      return 0;
    }
		if (param == "--marker")
		{
			if (((i+1)<((size_t)argc)))
			{
				i++;
				markerPath = (argv[i]);
			}else {std::cout << " Not enough parameters, opening default path " << markerPath << "\n";}

		}    else if(param == "qhd")
    {
      topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "hd")
    {
      topicColor = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "ir")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "sd")
    {
      topicColor = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
      topicDepth = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    }
    else if(param == "approx")
    {
      useExact = false;
    }

    else if(param == "compressed")
    {
      useCompressed = true;
    }
    else if(param == "image")
    {
      mode = Receiver::IMAGE;
    }
    else if(param == "cloud")
    {
      mode = Receiver::CLOUD;
    }
    else if(param == "both")
    {
      mode = Receiver::BOTH;
    }
    else
    {
      ns = param;
    }
  }

  topicColor = "/" + ns + topicColor;
  topicDepth = "/" + ns + topicDepth;
  OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
  OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

  Receiver receiver(topicColor, topicDepth, useExact, useCompressed, markerPath);

  OUT_INFO("starting receiver...");
  receiver.run(mode);

  ros::shutdown();
  return 0;
}
