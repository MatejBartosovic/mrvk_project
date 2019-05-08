//
// Created by jakub on 7.5.2019.
//

#include "AddWaypointDialog.h"
#include <ui_AddWaypointDialog.h>


namespace mrvk_gui {
    AddWaypointDialog::AddWaypointDialog(QWidget* parent) :
            QDialog(parent),
            ui(new Ui::AddWaypointDialog) {

        ui->setupUi(this);

        QRegExpValidator* validator = new QRegExpValidator(QRegExp("^(\\+|-)?(?:90(?:(?:\\.0{1,6})?)|(?:[0-9]|[1-8][0-9])(?:(?:\\.[0-9]{1,6})?))$"), this);
        ui->input1->setValidator(validator);
        ui->input2->setValidator(validator);
    }

    AddWaypointDialog::~AddWaypointDialog() {
        delete ui;
    }

    void AddWaypointDialog::on_rbtnRobot_clicked() {
//        osm_planner::coordinates_converters::GeoNode geoNode;

    }

    void AddWaypointDialog::on_rbtnGps_clicked() {

    }

    void AddWaypointDialog::on_btnOk_clicked() {
        if (ui->input1->text().isEmpty() || ui->input2->text().isEmpty()) {
            QMessageBox msgBox(QMessageBox::Critical, "Error", "Input is empty");
            msgBox.exec();
            return;
        }

        latitudeGoalValue = ui->input1->text();
        longitudeGoalValue = ui->input2->text();
        close();
    }

    void AddWaypointDialog::on_btnCancel_clicked() {
        done(-1);
    }

    QGeoPose AddWaypointDialog::getGeoPose() {
        QGeoPose geoPose;
        geoPose.latitude = latitudeGoalValue;
        geoPose.longitude = longitudeGoalValue;
        return geoPose;
    }

    int AddWaypointDialog::execInitGeoPose(const QString& latitude, const QString& longitude) {
        ui->input1->setText(latitude);
        ui->input2->setText(longitude);
        return exec();
    }
}