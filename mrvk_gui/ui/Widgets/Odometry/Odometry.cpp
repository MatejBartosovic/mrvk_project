#include "Odometry.h"
#include <ui_Odometry.h>

using namespace std;

namespace mrvk_gui {
    Odometry::Odometry(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::Odometry) {

        ui->setupUi(this);
        ros::NodeHandle n("/");
        odom_subscriber = new Subscriber<nav_msgs::Odometry>("odom",n);
    }

    Odometry::~Odometry() {
        delete odom_subscriber;
        delete ui;
    }

    void mrvk_gui::Odometry::updateData() {
        auto data = odom_subscriber->getData();
        ui->valueXvelocity->setNum(data.twist.twist.linear.x);
        ui->valueYawVelocity->setNum(data.twist.twist.angular.z);
    }

}