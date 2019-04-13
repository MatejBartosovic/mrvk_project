#include "Gps.h"
#include <ui_Gps.h>

namespace mrvk_gui{
    Gps::Gps(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Gps)
    {
        ui->setupUi(this);
        ros::NodeHandle n("/");
        subscriber = new Subscriber<gps_common::GPSFix>("/gps",n);
    }

    Gps::~Gps()
    {
        delete ui;
    }

    void Gps::updateData(){
        auto data = subscriber->getData();
        ui->latitudeValue->setText(QString::number(data.latitude, 'f', 4));
        ui->longitudeValue->setText(QString::number(data.longitude, 'f', 4));
        ui->altitudeValue->setText(QString::number(data.altitude, 'f', 4));
        ui->latituteAccuracyValue->setText(QString::number(data.position_covariance[0], 'f', 4));
        ui->logitudeAccuracyValue->setText(QString::number(data.position_covariance[4], 'f', 4));
        ui->altitudeAccuracyValue->setText(QString::number(data.position_covariance[8], 'f', 4));
    }
}
