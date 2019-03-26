#include "Gps.h"
#include <ui_Gps.h>

namespace mrvk_gui{
    Gps::Gps(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Gps)
    {
        ui->setupUi(this);
        ros::NodeHandle n("/");
        subscriber = new Subscriber<gps_common::GPSFix>("/fix",n);
    }

    Gps::~Gps()
    {
        delete ui;
    }

    void Gps::updateData(){
        auto data = subscriber->getData();
        ui->latitudeValue->setNum(data.latitude);
        ui->longitudeValue->setNum(data.longitude);
        ui->altitudeValue->setNum(data.altitude);
        ui->latituteAccuracyValue->setNum(data.position_covariance[0]);
        ui->logitudeAccuracyValue->setNum(data.position_covariance[4]);
        ui->altitudeAccuracyValue->setNum(data.position_covariance[8]);
    }
}
