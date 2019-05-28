//
// Created by jakub on 7.5.2019.
//

#include "AddWaypointDialog.h"
#include <ui_AddWaypointDialog.h>
#include <move_base_msgs/MoveBaseGoal.h>


namespace mrvk_gui {
    AddWaypointDialog::AddWaypointDialog(QWidget* parent) :
            QDialog(parent),
            ui(new Ui::AddWaypointDialog) {

        ui->setupUi(this);

        QRegExpValidator* validator = new QRegExpValidator(QRegExp("^(\\+|-)?(?:90(?:(?:\\.0{1,6})?)|(?:[0-9]|[1-8][0-9])(?:(?:\\.[0-9]{1,15})?))$"), this);
        ui->input1->setValidator(validator);
        ui->input2->setValidator(validator);

        ros::NodeHandle n;
        double latOffset, longOffset;
        n.param<double>(ORIGIN_LATITUDE_PARAM_PATH, latOffset, 0);
        n.param<double>(ORIGIN_LONGTITUDE_PARAM_PATH, longOffset, 0);
        converter.setOrigin(latOffset, longOffset);
    }

    AddWaypointDialog::~AddWaypointDialog() {
        delete ui;
    }

    void AddWaypointDialog::on_rbtnRobot_clicked() {
        if (!gpsSpaceSelection) {
            return;
        } else {
            gpsSpaceSelection = false;
        }
        ui->inputLabel1->setText("X");
        ui->inputLabel2->setText("Y");

        double latitude = ui->input1->text().toDouble();
        double longitude = ui->input2->text().toDouble();

        auto mapPose = gps2mapPose(GpsPose{latitude, longitude});

        ui->input1->setText(QString::number(mapPose.x, 'f', 12));
        ui->input2->setText(QString::number(mapPose.y, 'f', 12));
    }

    void AddWaypointDialog::on_rbtnGps_clicked() {
        if (gpsSpaceSelection) {
            return;
        } else {
            gpsSpaceSelection = true;
        }
        ui->inputLabel1->setText("Latitude");
        ui->inputLabel2->setText("Longitude");

        double x = ui->input1->text().toDouble();
        double y = ui->input2->text().toDouble();

        GpsPose gpsPose;
        try {
            gpsPose = map2gpsPose(MapPose{x, y});
        } catch (std::runtime_error &e) {
            return;
        }

        ui->input1->setText(QString::number(gpsPose.latitude, 'f', 12));
        ui->input2->setText(QString::number(gpsPose.longitude, 'f', 12));
    }

    void AddWaypointDialog::on_btnOk_clicked() {
        if (ui->input1->text().isEmpty() || ui->input2->text().isEmpty()) {
            QMessageBox msgBox(QMessageBox::Critical, "Error", "Input is empty");
            msgBox.exec();
            return;
        }

        if (gpsSpaceSelection) {
            latitudeGoalValue = ui->input1->text();
            longitudeGoalValue = ui->input2->text();
        } else {
            double x = ui->input1->text().toDouble();
            double y = ui->input2->text().toDouble();

            GpsPose gpsPose;
            try {
                gpsPose = map2gpsPose(MapPose{x, y});
            } catch (std::runtime_error &e) {
                close();
                return;
            }
            latitudeGoalValue = QString::number(gpsPose.latitude, 'f', 12);
            longitudeGoalValue = QString::number(gpsPose.longitude, 'f', 12);
        }
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

    MapPose AddWaypointDialog::gps2mapPose(GpsPose geoPose) {
        double x = -converter.getCoordinateX(geoPose.latitude, geoPose.longitude);
        double y = -converter.getCoordinateY(geoPose.latitude, geoPose.longitude);

        return {x, y};
    }

    GpsPose AddWaypointDialog::map2gpsPose(MapPose mapPose) {
        tf::TransformListener listener;
        tf::StampedTransform baseLinkWorldTF;

        try{
            listener.lookupTransform(TF_BASE_LINK_FRAME, TF_WORLD_FRAME, ros::Time(0), baseLinkWorldTF);
        } catch (tf::TransformException &ex){
            std::ostringstream oss;
            oss << "Can't find transform between child [" << TF_BASE_LINK_FRAME <<"] and robot base_link frame ["
                << TF_WORLD_FRAME << "]. Exception:" << ex.what();
            ROS_ERROR(oss.str().c_str());
            QMessageBox box(QMessageBox::Critical, "Tf", oss.str().c_str());
            box.exec();
            throw std::runtime_error("Can not transform map to gps pose");
        }

        tf::Transform result = baseLinkWorldTF * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(mapPose.x, mapPose.y, 0));

        auto origin = result.getOrigin();
        double latitude = converter.getLatitude(origin.x(), origin.y());
        double longitude = converter.getLongitude(origin.x(), origin.y());

        return {latitude, longitude};
    }
}