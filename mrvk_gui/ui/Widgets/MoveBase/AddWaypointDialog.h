//
// Created by jakub on 7.5.2019.
//

#ifndef ADDWAYPOINTDIALOG_H
#define ADDWAYPOINTDIALOG_H

#include <QDialog>
#include <QMessageBox>
#include <mrvk_gui/GuiDefines.h>
#include <osm_planner/coordinates_converters/haversine_formula.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <mrvk_gui_interface/AddGeoWaypoint.h>
#include <mrvk_gui_interface/EditWaypoint.h>
//#include <osm_planner/coordinates_converters/haversine_formula.h>

namespace Ui {
    class AddWaypointDialog;
}

namespace mrvk_gui {
    typedef struct {
        double x;
        double y;
    } MapPose;

    typedef struct {
        double latitude;
        double longitude;
    } GpsPose;

    class AddWaypointDialog : public QDialog {
    Q_OBJECT
    public:
        explicit AddWaypointDialog(QWidget* parent = 0);

        ~AddWaypointDialog() override;

        int execInitGeoPose(const QString& latitude, const QString& longitude);

        GpsPose getGpsPose();

//        void initGeoPose(GeoPose pose);


    private Q_SLOTS:
//        void on_pushButton_clicked();
        void on_rbtnRobot_clicked();
        void on_rbtnGps_clicked();
        void on_btnOk_clicked();
        void on_btnCancel_clicked();

    private:

        ros::ServiceClient addWaypointSrv;
        ros::ServiceClient editWaypointSrv;

        MapPose gps2mapPose(GpsPose geoPose);
        GpsPose map2gpsPose(MapPose mapPose);

        osm_planner::coordinates_converters::HaversineFormula converter;
        bool gpsSpaceSelection = true;
        double latitudeGoalValue;
        double longitudeGoalValue;

        Ui::AddWaypointDialog* ui;
    };
}
#endif //ADDWAYPOINTDIALOG_H
