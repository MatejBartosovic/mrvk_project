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
        //gpsFrequency = new TopicFrequencySubscriber<nav_msgs::Odometry>("/odom",n);
        //lidarFrequency = new TopicFrequencySubscriber<nav_msgs::Odometry>("/odom",n);
        cameraFrequency = new TopicFrequencySubscriber<sensor_msgs::Image>("/mrvk_gui/image",n);
    }

    TopicFrequency::~TopicFrequency() {
        delete ui;
    }

    void TopicFrequency::updateData(){
        //QString s;
        //s = s.setNum(cameraFrequency->getFrequency(), 'g', 6);

        ui->odometryValue->setNum(odometryFrequency->getFrequency());
        ui->imuValue->setNum(imuFrequency->getFrequency());
        //ui->gpsValue->setNum(gpsFrequency->getFrequency());
        //ui->lidarValue->setNum(lidarFrequency->getFrequency());
        ui->cameraValue->setNum(cameraFrequency->getFrequency());
    }
}