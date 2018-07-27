/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "qr_detector/qr_detector_nodelet.h"

#include "pluginlib/class_list_macros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
//#include "std_msgs/Bool.h"
#include "qr_detector/StartScanQR.h"
bool scan_QR = false;

PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

namespace qr_detector {

//    void startScanCallback(const std_msgs::Bool::ConstPtr& msg)
//    {
//        scan_QR = true;
//        NODELET_INFO("Start scan QR.");
//    }
    bool startScanQR(qr_detector::StartScanQR::Request  &req,
                     qr_detector::StartScanQR::Response &res)
    {

        return true;
    }

    QrDetectorNodelet::QrDetectorNodelet()
        : it(nh)
    { }

    QrDetectorNodelet::~QrDetectorNodelet()
    {
        imgSubscriber.shutdown();
    }

    void QrDetectorNodelet::onInit()
    {
        nh = getNodeHandle();

        tagsPublisher = nh.advertise<std_msgs::String>("qr_codes", 10,
                                                       boost::bind(&QrDetectorNodelet::connectCb, this),
                                                       boost::bind(&QrDetectorNodelet::disconnectCb, this));

        NODELET_INFO_STREAM("Initialising nodelet... [" << nh.getNamespace() << "]");
    }

    void QrDetectorNodelet::connectCb()
    {
        if (!imgSubscriber && tagsPublisher.getNumSubscribers() > 0)
        {
            NODELET_INFO("Connecting to image topic.");
            imgSubscriber = it.subscribe("image", 1, &QrDetectorNodelet::imageCb, this);
            //ros::Subscriber startScanSubscriber = it.subscribe("start_scan_QR", 1, startScanCallback);
            ros::ServiceServer service = it.advertiseService("add_two_ints", add);

        }
    }

    void QrDetectorNodelet::disconnectCb()
    {
        if (tagsPublisher.getNumSubscribers() == 0)
        {
            NODELET_INFO("Unsubscribing from image topic.");
            imgSubscriber.shutdown();
        }
    }

    void QrDetectorNodelet::imageCb(const sensor_msgs::ImageConstPtr &image)
    {
        if(scan_QR)
        {
            cv_bridge::CvImageConstPtr cv_image;

            try
            {
                cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            auto tags = detector.detect(cv_image->image, 10);
            for (auto& tag : tags)
            {
                tagsPublisher.publish(tag.message);
            }
        }
    }
}
