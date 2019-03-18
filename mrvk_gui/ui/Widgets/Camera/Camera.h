#ifndef CAMERA_H
#define CAMERA_H

#include <QWidget>

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

    private:
        Ui::Camera *ui;
    };
}

#endif // CAMERA_H
