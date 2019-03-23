#ifndef TOPICFREQUENCY_H
#define TOPICFREQUENCY_H

#include <QWidget>
#include <mrvk_gui/TopicFrequency.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

namespace Ui {
    class TopicFrequency;
}
namespace mrvk_gui {

    class TopicFrequency : public QWidget {
    Q_OBJECT

    public:
        explicit TopicFrequency(QWidget* parent = 0);

        ~TopicFrequency();
        void updateData();

    private:
        Ui::TopicFrequency* ui;
        TopicFrequencySubscriber<nav_msgs::Odometry>* odometryFrequency;
        TopicFrequencySubscriber<sensor_msgs::Imu>* imuFrequency;
        //TopicFrequencySubscriber<nav_msgs::Odometry>* gpsFrequency;
        //TopicFrequencySubscriber<nav_msgs::Odometry>* lidarFrequency;
        TopicFrequencySubscriber<sensor_msgs::Image>* cameraFrequency;
    };
}
#endif // TOPICFREQUENCY_H
