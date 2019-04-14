#ifndef TOPICFREQUENCY_H
#define TOPICFREQUENCY_H

#include <QWidget>
#include <mrvk_gui/TopicFrequency.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <gps_common/GPSFix.h>
#include <QLabel>
#include <mrvk_gui/GuiDefines.h>

#define HZ_OK 1
#define HZ_WARNING 0.5

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
        TopicFrequencySubscriber<gps_common::GPSFix>* gpsFrequency;
        TopicFrequencySubscriber<sensor_msgs::LaserScan>* lidarFrequency;
        TopicFrequencySubscriber<sensor_msgs::Image>* cameraFrequency;

        void setLabel(QLabel *label, double value);
    };
}
#endif // TOPICFREQUENCY_H
