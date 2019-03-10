#include "MoveBaseStatus.h"
#include <ui_MoveBaseStatus.h>

namespace mrvk_gui {
    MoveBaseStatus::MoveBaseStatus(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBaseStatus) {
        ui->setupUi(this);
    }

    MoveBaseStatus::~MoveBaseStatus() {
        delete ui;
    }
}