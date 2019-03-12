#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <QWidget>
#include <mrvk_gui/Subscriber.h>
#include <nav_msgs/Odometry.h>

namespace Ui {
class Odometry;
}

namespace mrvk_gui {
    class Odometry : public QWidget {
    Q_OBJECT

    public:
        explicit Odometry(QWidget* parent = 0);

        ~Odometry() override;
        void updateData();

    private:
        Ui::Odometry* ui;
        mrvk_gui::Subscriber<nav_msgs::Odometry> *odom_subscriber;
    };
}
#endif // ODOMETRY_H
