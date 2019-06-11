//
// Created by jakub on 7.5.2019.
//

#ifndef WAYPOINTSTABLEWIDGET_H
#define WAYPOINTSTABLEWIDGET_H

#include <QWidget>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <gps_common/GPSFix.h>
#include <mrvk_gui/GuiDefines.h>
#include <mrvk_gui/Subscriber.h>
#include "AddWaypointDialog.h"
#include <list>
#include <mrvk_gui_interface/GetWaypointsQueue.h>
#include <mrvk_gui_interface/AddGeoWaypoint.h>

namespace Ui {
    class WaypointsTableWidget;
}

namespace mrvk_gui {
    typedef struct {
        double x;
        double y;
    } Waypoint;


    class WaypointsTableWidget : public QWidget {
    Q_OBJECT

    public:
        explicit WaypointsTableWidget(QWidget* parent = 0);
        ~WaypointsTableWidget() override;

        std::list<Waypoint> getWaypoints();

    private slots:
        void on_btnAddNew_clicked();
        void on_btnRemove_clicked();
        void on_btnEdit_clicked();
        void on_btnAddCurrent_clicked();

    private:
        void addWaypoint(bool active, const double& latitude, const double& longitude);
        void editWaypoint(int row, const QString& latitude, const QString& longitude);

        void updateWaypoints();
        void showErrorMessage(const QString& message);

        QWidget* parent;
        Ui::WaypointsTableWidget* ui;
        Subscriber<gps_common::GPSFix> *subscriber;

        ros::ServiceClient getWaypointsSrv;
        ros::ServiceClient addWaypointSrv;
        ros::ServiceClient editWaypointSrv;
    };
}


#endif //MOVEWAYPOINTS_H
