#include "Camera.h"
#include <ui_Camera.h>
#include <QImage>

namespace mrvk_gui {
    Camera::Camera(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::Camera) {
        ui->setupUi(this);
        ros::NodeHandle n("~");
        subscriber = new Subscriber<sensor_msgs::Image>("image",n);
    }

    Camera::~Camera() {
        delete ui;
        delete subscriber;
    }

    void Camera::updateData(){
        auto image = subscriber->getData();

        QImage qImage(image.data.data(),image.width,image.height,QImage::Format::Format_RGB16); //TODO image format
        QPixmap pixmap;
        pixmap.fromImage(qImage);
        ui->image->setPixmap(pixmap);

    }
}