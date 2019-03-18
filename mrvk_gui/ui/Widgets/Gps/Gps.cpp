#include "Gps.h"
#include <ui_Gps.h>

namespace mrvk_gui{
    Gps::Gps(QWidget *parent) :
        QWidget(parent),
        ui(new Ui::Gps)
    {
        ui->setupUi(this);
    }

    Gps::~Gps()
    {
        delete ui;
    }
}
