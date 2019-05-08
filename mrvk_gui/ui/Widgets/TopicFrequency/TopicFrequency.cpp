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
        odometryFrequency = new TopicFrequencySubscriber<nav_msgs::Odometry>(mrvk_gui::topics::odometry,n);
        imuFrequency = new TopicFrequencySubscriber<sensor_msgs::Imu>(mrvk_gui::topics::imu,n);
        gpsFrequency = new TopicFrequencySubscriber<gps_common::GPSFix>(mrvk_gui::topics::gps,n);
        lidarFrequency = new TopicFrequencySubscriber<sensor_msgs::LaserScan>(mrvk_gui::topics::scan,n);
        cameraFrequency = new TopicFrequencySubscriber<sensor_msgs::Image>(mrvk_gui::topics::camera,n);
    }

    TopicFrequency::~TopicFrequency() {
        delete ui;
        delete odometryFrequency;
        delete imuFrequency;
        delete gpsFrequency;
        delete lidarFrequency;
        delete cameraFrequency;
    }

    void TopicFrequency::updateData(){
        setLabel(ui->odometryValue, odometryFrequency->getFrequency());
        setLabel(ui->imuValue, imuFrequency->getFrequency());
        setLabel(ui->gpsValue, gpsFrequency->getFrequency());
        setLabel(ui->lidarValue, lidarFrequency->getFrequency());
        setLabel(ui->cameraValue, cameraFrequency->getFrequency());
    }

    void TopicFrequency::setLabel(QLabel *label, double value) {
        if (HZ_OK < value) {
            label->setStyleSheet(QLABEL_COLOR_OK);
        } else if (HZ_WARNING < value) {
            label->setStyleSheet(QLABEL_COLOR_WARNING);
        } else {
            label->setStyleSheet(QLABEL_COLOR_ERROR);
        }

        label->setText(QString::number(value, 'f', 4));
    }
}

