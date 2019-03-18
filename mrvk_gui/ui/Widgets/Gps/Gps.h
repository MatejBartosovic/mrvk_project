#ifndef GPS_H
#define GPS_H

#include <QWidget>

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

    private:
        Ui::Gps *ui;
    };
}

#endif // GPS_H
