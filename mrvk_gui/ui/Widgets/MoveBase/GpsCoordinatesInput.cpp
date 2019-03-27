#include "GpsCoordinatesInput.h"
#include "ui_GpsCoordinatesInput.h"

GpsCoordinatesInput::GpsCoordinatesInput(double latitude, double longitude, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GpsCoordinatesInput)
{
    ui->setupUi(this);
    std::lconv* lc = std::localeconv();
    QRegExpValidator* latitudeValidator = new QRegExpValidator(QRegExp("^(\\+|-)?(?:90(?:(?:\\.0{1,6})?)|(?:[0-9]|[1-8][0-9])(?:(?:\\" + QString( lc->decimal_point) + "[0-9]{1,6})?))$"));
    ui->latitudeValue->setValidator(latitudeValidator);

    QRegExpValidator* longitudeValidator = new QRegExpValidator(QRegExp("^(\\+|-)?(?:180(?:(?:\\.0{1,6})?)|(?:[0-9]|[1-9][0-9]|1[0-7][0-9])(?:(?:\\"+ QString( lc->decimal_point) + "[0-9]{1,6})?))$"));
    ui->longitudeValue->setValidator(longitudeValidator);
    setLatitude(latitude);
    setLongitude(longitude);
}

void GpsCoordinatesInput::setLatitude(double latiude){
    ui->latitudeValue->setText(std::to_string(latiude).c_str());
}
void GpsCoordinatesInput::setLongitude(double longitude){
    ui->longitudeValue->setText(std::to_string(longitude).c_str());
}

double GpsCoordinatesInput::getLatitude(){
    return std::stod(ui->latitudeValue->text().toStdString());
}
double GpsCoordinatesInput::getLongitude(){
    return std::stod(ui->longitudeValue->text().toStdString());
}

GpsCoordinatesInput::~GpsCoordinatesInput()
{
    delete ui;
}
