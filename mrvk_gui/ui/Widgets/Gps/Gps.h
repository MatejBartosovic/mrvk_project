#ifndef GPS_H
#define GPS_H

#include <QWidget>
#include <mrvk_gui/Subscriber.h>
#include <gps_common/GPSFix.h>

namespace Ui {
class Gps;
}

namespace mrvk_gui{
    class Gps : public QWidget
    {
        Q_OBJECT

    public:
        explicit Gps(QWidget *parent = 0);
        ~Gps();

        void updateData();

    private:
        Ui::Gps *ui;
        Subscriber<gps_common::GPSFix> *subscriber;
    };
}

#endif // GPS_H
