#include "MrvkDriver.h"
#include <ui_MrvkDriver.h>

namespace mrvk_gui {
    MrvkDriver::MrvkDriver(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MrvkDriver()) {
        ui->setupUi(this);
    }

    MrvkDriver::~MrvkDriver() {
        delete ui;
    }
}