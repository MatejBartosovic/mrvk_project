#include "MoveBase.h"
#include <ui_MoveBase.h>

namespace mrvk_gui {
    MoveBase::MoveBase(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBase) {
        ui->setupUi(this);
    }

    MoveBase::~MoveBase() {
        delete ui;
    }
}