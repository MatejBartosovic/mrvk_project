#include "TopicFrequency.h"
#include "ui_TopicFrequency.h"
#include <ros/ros.h>
#include <mrvk_gui/TopicFrequency.h>

namespace mrvk_gui {
    TopicFrequency::TopicFrequency(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::TopicFrequency){
        ui->setupUi(this);
        ros::NodeHandle n("/");
        odometryFrequency = new TopicFrequencySubscriber<nav_msgs::Odometry>("/odom",n);
        imuFrequency = new TopicFrequencySubscriber<sensor_msgs::Imu>("/adis16488/imu_data",n);
        gpsFrequency = new TopicFrequencySubscriber<gps_common::GPSFix>("/gps",n);
        lidarFrequency = new TopicFrequencySubscriber<sensor_msgs::LaserScan>("/scan",n);
        cameraFrequency = new TopicFrequencySubscriber<sensor_msgs::Image>("/mrvk_gui/image",n);


    }

    TopicFrequency::~TopicFrequency() {
        delete ui;
    }

    void TopicFrequency::updateData(){
        ui->odometryValue->setText(QString::number(odometryFrequency->getFrequency(), 'f', 4));
        ui->imuValue->setText(QString::number(imuFrequency->getFrequency(), 'f', 4));
        ui->gpsValue->setText(QString::number(gpsFrequency->getFrequency(), 'f', 4));
        ui->lidarValue->setText(QString::number(lidarFrequency->getFrequency(), 'f', 4));
        ui->cameraValue->setText(QString::number(cameraFrequency->getFrequency(), 'f', 4));
    }
}
