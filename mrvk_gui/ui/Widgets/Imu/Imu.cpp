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
        ui->axValue->setText(QString::number(newData.linear_acceleration.x, 'f', 4));
        ui->ayValue->setText(QString::number(newData.linear_acceleration.y, 'f', 4));
        ui->azValue->setText(QString::number(newData.linear_acceleration.z, 'f', 4));

        ui->gxValue->setText(QString::number(newData.angular_velocity.x, 'f', 4));
        ui->gyValue->setText(QString::number(newData.angular_velocity.y, 'f', 4));
        ui->gzValue->setText(QString::number(newData.angular_velocity.z, 'f', 4));

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
