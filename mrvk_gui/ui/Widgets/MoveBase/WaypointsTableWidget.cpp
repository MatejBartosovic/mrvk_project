//
// Created by jakub on 7.5.2019.
//

#include "WaypointsTableWidget.h"
#include <ui_WaypointsTableWidget.h>

//TODO ked pride error respones otvor msgbox
//TODO update waypoints musi urobit clear waypointov v UI

namespace mrvk_gui {
    WaypointsTableWidget::WaypointsTableWidget(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::WaypointsTableWidget) {
        ui->setupUi(this);

        this->parent = parent;

        // TODO mozno by bolo lepsie citat polohu z odometrie
        ros::NodeHandle n("/");
        subscriber = new Subscriber<gps_common::GPSFix>(mrvk_gui::topics::gps,n);

        ui->tableWidget->setColumnCount(3);
        ui->tableWidget->setColumnWidth(0, 27);
        ui->tableWidget->setColumnWidth(1, 85);
        ui->tableWidget->setColumnWidth(2, 85);
        ui->tableWidget->setShowGrid(true);
        ui->tableWidget->setSelectionMode(QAbstractItemView::SingleSelection);
        ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
        ui->tableWidget->setHorizontalHeaderLabels(QStringList({"", "Latitude", "Longitude"}));
        ui->tableWidget->horizontalHeader()->setStretchLastSection(true);


        addWaypointSrv = n.serviceClient<mrvk_gui_interface::AddGeoWaypoint>("mrvk_gui_interface/add_waypoint");
        editWaypointSrv = n.serviceClient<mrvk_gui_interface::EditWaypoint>("mrvk_gui_interface/edit_waypoint");
        getWaypointsSrv = n.serviceClient<mrvk_gui_interface::GetWaypointsQueue>("mrvk_gui_interface/get_waypoints");

        updateWaypoints();

    }

    WaypointsTableWidget::~WaypointsTableWidget() {
//        saveWaypointsToFile(WAYPOINTS_TABLE_CONFIG_FILE);
        delete ui;
    }

    void WaypointsTableWidget::addWaypoint(bool active, const double& latitude, const double& longitude) {
        unsigned int row = ui->tableWidget->rowCount();
        ui->tableWidget->insertRow(row);

        auto *item = new QTableWidgetItem();
        item->data(Qt::CheckStateRole);
        if (active) {
            item->setCheckState(Qt::Checked);
        } else {
            item->setCheckState(Qt::Unchecked);
        }
        ui->tableWidget->setItem(row,0, item);

        item = new QTableWidgetItem(QString::number(latitude, 'f', 6));
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        ui->tableWidget->setItem(row,1, item);

        item = new QTableWidgetItem(QString::number(longitude, 'f', 6));
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        ui->tableWidget->setItem(row,2, item);
    }

    void WaypointsTableWidget::editWaypoint(int row, const QString& latitude, const QString& longitude) {
        auto item = ui->tableWidget->item(row, 1);
        item->setText(latitude);
        item = ui->tableWidget->item(row, 2);
        item->setText(longitude);
    }


    void WaypointsTableWidget::on_btnAddNew_clicked() {
        mrvk_gui::AddWaypointDialog addNew(this);
        int dialogCode = addNew.exec();

        if (dialogCode == 0) {
            mrvk_gui::GpsPose gpsPose = addNew.getGpsPose();
           // qDebug(gpsPose.latitude.toLatin1() + ", " + gpsPose.longitude.toLatin1());

            mrvk_gui_interface::AddGeoWaypoint srv;
            srv.request.waypoint.latitude = gpsPose.latitude;
            srv.request.waypoint.longitude = gpsPose.longitude;
            srv.request.waypoint.active = true;
            srv.request.index = -1;
            addWaypointSrv.call(srv);

        } else {
            qDebug("Cancel");
        }

        updateWaypoints();
    }

    void WaypointsTableWidget::on_btnRemove_clicked() {
        int row = ui->tableWidget->currentRow();
        if (row < 0) {
            showErrorMessage("Select row before");
            return;
        }
        ui->tableWidget->removeRow(row);
    }

    void WaypointsTableWidget::on_btnEdit_clicked() {
        int row = ui->tableWidget->currentRow();
        if (row < 0) {
            showErrorMessage("Select waypoint before edit");
            return;
        }

        mrvk_gui::AddWaypointDialog addNew(this);

        QString latitude = ui->tableWidget->item(row, 1)->text();
        QString longitude = ui->tableWidget->item(row, 2)->text();

        int dialogCode = addNew.execInitGeoPose(latitude, longitude);  // edit waypoint

        if (dialogCode == 0) {
            mrvk_gui::GpsPose gpsPose = addNew.getGpsPose();

            mrvk_gui_interface::EditWaypoint srv;
            srv.request.waypoint.latitude = gpsPose.latitude;
            srv.request.waypoint.longitude = gpsPose.longitude;
            srv.request.waypoint.active = true;         // todo read checkbox
            srv.request.index = row;
            editWaypointSrv.call(srv);

        } else {
            qDebug("Cancel");
        }

        updateWaypoints();
    }

    void WaypointsTableWidget::on_btnAddCurrent_clicked() {
        auto data = subscriber->getData();

        mrvk_gui_interface::AddGeoWaypoint srv;
        srv.request.waypoint.latitude = data.latitude;
        srv.request.waypoint.longitude = data.longitude;
        srv.request.waypoint.active = true;
        srv.request.index = -1;
        addWaypointSrv.call(srv);

        updateWaypoints();
    }

    void WaypointsTableWidget::showErrorMessage(const QString& message){
        QMessageBox msgBox(QMessageBox::Critical, "Error", message);
        msgBox.exec();
    }

//    std::list<Waypoint> WaypointsTableWidget::getWaypoints() {
//        std::list<Waypoint> way;
//        QTableWidgetItem *item;
//
//        for (int i = 0; i < ui->tableWidget->rowCount(); i++) {
//            item = ui->tableWidget->item(i, 0);
//            if (item->checkState() == Qt::Checked) {
//                Waypoint point;
//                //TODO body treba prekonvertovat z Geo na Map
//                point.x = ui->tableWidget->item(i, 1)->text().toDouble();
//                point.y = ui->tableWidget->item(i, 2)->text().toDouble();
//                way.push_back(point);
//            }
//        }
//
//        return way;
//    }

    void WaypointsTableWidget::updateWaypoints() {
        mrvk_gui_interface::GetWaypointsQueue srv;
        getWaypointsSrv.call(srv);
        auto waypoints = srv.response.waypoints;

        for (const auto point: waypoints) {
            addWaypoint(point.active, point.latitude, point.longitude);
        }
    }
}