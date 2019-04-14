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
        ui->latitudeValue->setText(QString::number(data.latitude, 'f', 6));
        ui->longitudeValue->setText(QString::number(data.longitude, 'f', 6));
        ui->altitudeValue->setText(QString::number(data.altitude, 'f', 4));

        ui->latituteAccuracyValue->setText(QString::number(data.position_covariance[0], 'f', 4));
        ui->logitudeAccuracyValue->setText(QString::number(data.position_covariance[4], 'f', 4));
        ui->altitudeAccuracyValue->setText(QString::number(data.position_covariance[8], 'f', 4));

        setLabelColor(ui->latituteAccuracyValue, data.position_covariance[0]);
        setLabelColor(ui->logitudeAccuracyValue, data.position_covariance[4]);
        setLabelColor(ui->altitudeAccuracyValue, data.position_covariance[8]);
    }

    void Gps::setLabelColor(QLabel* label, double value) {
        if (ACCURACY_BAD < value) {
            label->setStyleSheet(QLABEL_COLOR_ERROR);
        } else if (ACCURACY_WARNING < value) {
            label->setStyleSheet(QLABEL_COLOR_WARNING);
        } else {
            label->setStyleSheet(QLABEL_COLOR_OK);
        }
    }
}
