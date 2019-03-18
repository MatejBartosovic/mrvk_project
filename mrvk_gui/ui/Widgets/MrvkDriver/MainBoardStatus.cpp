#include "MainBoardStatus.h"
#include <ui_MainBoardStatus.h>

namespace mrvk_gui {
    MainBoardStatus::MainBoardStatus(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MainBoardStatus()) {
        ui->setupUi(this);
    }

    MainBoardStatus::~MainBoardStatus() {
        delete ui;
    }
}