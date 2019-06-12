//
// Created by jakub on 7.5.2019.
//

#ifndef WAYPOINTSTABLEWIDGET_H
#define WAYPOINTSTABLEWIDGET_H

// ros
#include <ros/ros.h>

//std
#include <list>

// Qt
#include <QWidget>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

// msgs
#include <mrvk_gui_interface/GetWaypoints.h>
#include <mrvk_gui_interface/AddGeoWaypoint.h>
#include <mrvk_gui_interface/EraseWaypoint.h>
#include <mrvk_gui_interface/SwapWaypoints.h>
#include <mrvk_gui_interface/EditWaypoint.h>
#include <gps_common/GPSFix.h>

// this gui
#include <mrvk_gui/GuiDefines.h>
#include <mrvk_gui/Subscriber.h>
#include "AddWaypointDialog.h"


namespace Ui {
    class WaypointsTableWidget;
}

namespace mrvk_gui {
    class WaypointsTableWidget : public QWidget {
    Q_OBJECT

    public:
        explicit WaypointsTableWidget(QWidget* parent = 0);
        ~WaypointsTableWidget() override;

    private slots:
        void on_btnAddNew_clicked();
        void on_btnRemove_clicked();
        void on_btnEdit_clicked();
        void on_btnAddCurrent_clicked();
        void on_btnMoveUp_clicked();
        void on_btnMoveDown_clicked();
        void on_btnSyncTable_clicked();
        void tableClicked(int row, int col);

    private:
        void addTableWaypoint(bool active, const double& latitude, const double& longitude);
        void showErrorMessage(const std::string& message);

        void updateWaypoints(const std::vector<mrvk_gui_interface::GeoPoint>& waypoints, int selectedRow = -1);
        void updateWaypoints(int selectedRow = -1);

        QWidget* parent;
        Ui::WaypointsTableWidget* ui;

        Subscriber<gps_common::GPSFix> *subscriber;
        ros::ServiceClient getWaypointsSrv;
        ros::ServiceClient addWaypointSrv;
        ros::ServiceClient editWaypointSrv;
        ros::ServiceClient eraseWaypointSrv;
        ros::ServiceClient swapWaypointSrv;
    };
}

#endif //MOVEWAYPOINTS_H
