#include "Imu.h"
#include <ui_Imu.h>

namespace mrvk_gui{
    Imu::Imu(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Imu)
    {
        ui->setupUi(this);
    }

    Imu::~Imu()
    {
        delete ui;
    }
}
