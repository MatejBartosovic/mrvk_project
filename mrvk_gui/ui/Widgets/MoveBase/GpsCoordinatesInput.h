#ifndef MAPOFFSETINPUT_H
#define MAPOFFSETINPUT_H

#include <QDialog>

namespace Ui {
class GpsCoordinatesInput;
}

class GpsCoordinatesInput : public QDialog
{
    Q_OBJECT

public:
    explicit GpsCoordinatesInput(double latitude = 0, double longitude = 0, QWidget *parent = 0);
    ~GpsCoordinatesInput();
    void setLatitude(double latiude);
    void setLongitude(double longitude);
    double getLatitude();
    double getLongitude();

private:
    Ui::GpsCoordinatesInput *ui;
};

#endif // MAPOFFSETINPUT_H
