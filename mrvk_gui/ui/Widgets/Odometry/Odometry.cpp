#include "Odometry.h"
#include <ui_Odometry.h>

namespace mrvk_gui {
    Odometry::Odometry(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::Odometry) {
        ui->setupUi(this);
    }

    Odometry::~Odometry() {
        delete ui;
    }
}