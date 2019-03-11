#ifndef IMU_H
#define IMU_H

#include <QWidget>
#include <mrvk_gui/Subscriber.h>
#include <sensor_msgs/Imu.h>


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
        void updateData();

    private:
        Ui::Imu *ui;
        mrvk_gui::Subscriber<sensor_msgs::Imu>* subscriber;
    };
}

#endif // IMU_H
