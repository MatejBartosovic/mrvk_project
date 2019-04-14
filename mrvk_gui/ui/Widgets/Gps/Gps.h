#ifndef GPS_H
#define GPS_H

#include <QWidget>
#include <QLabel>
#include <mrvk_gui/Subscriber.h>
#include <gps_common/GPSFix.h>
#include <mrvk_gui/GuiDefines.h>

#define ACCURACY_BAD 10
#define ACCURACY_WARNING 1

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
        void setLabelColor(QLabel *label, double value);

    };
}

#endif // GPS_H
