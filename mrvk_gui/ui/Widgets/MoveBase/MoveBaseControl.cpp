#include "MoveBaseControl.h"
#include <ui_MoveBaseControl.h>

namespace mrvk_gui {
    MoveBaseControl::MoveBaseControl(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBaseControl) {
        ui->setupUi(this);
    }

    MoveBaseControl::~MoveBaseControl() {
        delete ui;
    }

    void MoveBaseControl::updateData(){

    }
}