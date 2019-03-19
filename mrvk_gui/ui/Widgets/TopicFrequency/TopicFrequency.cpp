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
    }

    TopicFrequency::~TopicFrequency() {
        delete ui;
    }

    void TopicFrequency::updateData(){
        ui->odometryValue->setNum(odometryFrequency->getFrequency());
    }
}