#include "Imu.h"
#include <ui_Imu.h>
#include <sensor_msgs/Imu.h>

namespace mrvk_gui{
    Imu::Imu(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Imu)
    {
        ui->setupUi(this);
        ros::NodeHandle n("/adis16488");
        subscriber = new Subscriber<sensor_msgs::Imu>("imu_data",n);
    }

    void Imu::updateData(){
        sensor_msgs::Imu newData = subscriber->getData();
        ui->axValue->setNum(newData.linear_acceleration.x);
        ui->ayValue->setNum(newData.linear_acceleration.y);
        ui->azValue->setNum(newData.linear_acceleration.z);

        ui->gxValue->setNum(newData.angular_velocity.x);
        ui->gyValue->setNum(newData.angular_velocity.y);
        ui->gzValue->setNum(newData.angular_velocity.z);

    }
    Imu::~Imu()
    {
        delete ui;
        delete subscriber;
    }
}
