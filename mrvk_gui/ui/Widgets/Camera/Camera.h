#ifndef CAMERA_H
#define CAMERA_H

#include <QWidget>
#include <mrvk_gui/Subscriber.h>
#include <sensor_msgs/Image.h>

namespace Ui {
class Camera;
}
namespace mrvk_gui{
    class Camera : public QWidget
    {
        Q_OBJECT

    public:
        explicit Camera(QWidget *parent = 0);
        ~Camera();

        void updateData();

    private:
        Ui::Camera *ui;
        Subscriber<sensor_msgs::Image>* subscriber;
    };
}

#endif // CAMERA_H
