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
//#include <osm_planner/coordinates_converters/haversine_formula.h>

namespace Ui {
    class AddWaypointDialog;
}

namespace mrvk_gui {
    typedef struct _QGeoPose{
        QString latitude;
        QString longitude;
    } QGeoPose;

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
        AddWaypointDialog(QWidget* parent = 0);

        ~AddWaypointDialog() override;

        int execInitGeoPose(const QString& latitude, const QString& longitude);

        QGeoPose getGeoPose();

//        void initGeoPose(GeoPose pose);


    private Q_SLOTS:
//        void on_pushButton_clicked();
        void on_rbtnRobot_clicked();
        void on_rbtnGps_clicked();
        void on_btnOk_clicked();
        void on_btnCancel_clicked();

    private:

        MapPose gps2mapPose(GpsPose geoPose);
        GpsPose map2gpsPose(MapPose mapPose);

        osm_planner::coordinates_converters::HaversineFormula converter;
        bool gpsSpaceSelection = true;
        QString latitudeGoalValue;
        QString longitudeGoalValue;

//        QString result;
        Ui::AddWaypointDialog* ui;
    };
}
#endif //ADDWAYPOINTDIALOG_H
