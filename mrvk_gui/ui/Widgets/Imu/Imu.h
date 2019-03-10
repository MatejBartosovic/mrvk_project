#ifndef IMU_H
#define IMU_H

#include <QWidget>

namespace Ui {
class Imu;
}
namespace mrvk_gui{
    class Imu : public QWidget
    {
        Q_OBJECT

    public:
        explicit Imu(QWidget *parent = 0);
        ~Imu();

    private:
        Ui::Imu *ui;
    };
}

#endif // IMU_H
