#include "Camera.h"
#include <ui_Camera.h>

namespace mrvk_gui {
    Camera::Camera(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::Camera) {
        ui->setupUi(this);
    }

    Camera::~Camera() {
        delete ui;
    }
}