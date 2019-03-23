#include "Imu.h"
#include <ui_Imu.h>
#include <sensor_msgs/Imu.h>
#include <mrvk_gui/ImuButtons.h>
#include <mrvk_gui/ButtonException.h>
#include <QMessageBox>

namespace mrvk_gui{
    Imu::Imu(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Imu)
    {
        ui->setupUi(this);
        ros::NodeHandle n("/adis16488");
        subscriber = new Subscriber<sensor_msgs::Imu>("imu_data",n);
        imuButtons = new ImuButtons();
        connect(ui->calibrationButton,SIGNAL(released()),this,SLOT(calibrate()));
        connect(ui->resetButton,SIGNAL(released()),this,SLOT(reset()));
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
        delete imuButtons;
    }

    void Imu::calibrate(){
        QString string;
        try{
            imuButtons->calibrate();
            string = "Ok";
        }
        catch (ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Set central stop",string);
        msgBox.exec();
    }

    void Imu::reset(){
        QString string;
        try{
            imuButtons->reset();
            string = "Ok";
        }
        catch (ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Set central stop",string);
        msgBox.exec();
    }

}
