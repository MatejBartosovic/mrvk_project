//
// Created by jakub on 7.5.2019.
//

#include "WaypointsTableWidget.h"
#include <ui_WaypointsTableWidget.h>

namespace mrvk_gui {
    WaypointsTableWidget::WaypointsTableWidget(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::WaypointsTableWidget) {
        ui->setupUi(this);

        this->parent = parent;

        // check if mrvk_gui_interface node running
        if (!ros::service::exists("mrvk_gui_interface/add_waypoint", true)) {
            showErrorMessage("Node: mrvk_gui interface is not running");
        }


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
        getWaypointsSrv = n.serviceClient<mrvk_gui_interface::GetWaypoints>("mrvk_gui_interface/get_waypoints");
        eraseWaypointSrv = n.serviceClient<mrvk_gui_interface::EraseWaypoint>("mrvk_gui_interface/erase_waypoint");
        swapWaypointSrv = n.serviceClient<mrvk_gui_interface::SwapWaypoints>("mrvk_gui_interface/swap_waypoints");

        updateWaypoints();

        connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(tableClicked(int, int)));
    }

    WaypointsTableWidget::~WaypointsTableWidget() {
        delete ui;
    }

    void WaypointsTableWidget::showErrorMessage(std::string message) {
        if (message.empty()) {
            message = "Unknown error";
        }
        ROS_WARN("%s", message.c_str());
        QMessageBox msgBox(QMessageBox::Critical, "Error", QString::fromStdString(message));
        msgBox.exec();
    }

    void WaypointsTableWidget::updateWaypoints(const std::vector<mrvk_gui_interface::GeoPoint>& waypoints, int selectedRow) {
        int tmpRow = -1;

        if (selectedRow < 0) {
            tmpRow = ui->tableWidget->currentRow();
        }

        ui->tableWidget->setRowCount(0);

        for (const auto point: waypoints) {
            addTableWaypoint(point.active, point.latitude, point.longitude);
        }

        if (selectedRow < 0) {
            ui->tableWidget->selectRow(tmpRow);
        } else {
            ui->tableWidget->selectRow(selectedRow);
        }
    }

    void WaypointsTableWidget::updateWaypoints(int selectedRow) {
        mrvk_gui_interface::GetWaypoints srv;
        getWaypointsSrv.call(srv);
        auto waypoints = srv.response.waypoints;

        if (!srv.response.success) showErrorMessage(srv.response.message);
        updateWaypoints(waypoints, selectedRow);
    }

    void WaypointsTableWidget::addTableWaypoint(bool active, const double& latitude, const double& longitude) {
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

    void WaypointsTableWidget::on_btnAddNew_clicked() {
        mrvk_gui::AddWaypointDialog addNew(this);
        int dialogCode = addNew.exec();

        if (dialogCode == 0) {
            mrvk_gui::GpsPose gpsPose = addNew.getGpsPose();
            mrvk_gui_interface::AddGeoWaypoint srv;
            srv.request.waypoint.latitude = gpsPose.latitude;
            srv.request.waypoint.longitude = gpsPose.longitude;
            srv.request.waypoint.active = true;
            srv.request.index = -1;
            addWaypointSrv.call(srv);

            if (!srv.response.success) showErrorMessage(srv.response.message);
            updateWaypoints(srv.response.waypoints);
        } else {
            updateWaypoints();
        }
    }

    void WaypointsTableWidget::on_btnAddCurrent_clicked() {
        auto data = subscriber->getData();

        mrvk_gui_interface::AddGeoWaypoint srv;
        srv.request.waypoint.latitude = data.latitude;
        srv.request.waypoint.longitude = data.longitude;
        srv.request.waypoint.active = true;
        srv.request.index = -1;
        addWaypointSrv.call(srv);

        if (!srv.response.success) showErrorMessage(srv.response.message);
        updateWaypoints(srv.response.waypoints);
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
            // read checkbox
            auto item = ui->tableWidget->item(row, 0);
            bool active = item->checkState() == Qt::Checked;
            srv.request.waypoint.active = active;
            srv.request.index = row;
            editWaypointSrv.call(srv);

            if (!srv.response.success) showErrorMessage(srv.response.message);
            updateWaypoints(srv.response.waypoints);
        } else {
            updateWaypoints();
        }
    }

    void WaypointsTableWidget::on_btnRemove_clicked() {
        int row = ui->tableWidget->currentRow();
        if (row < 0) {
            showErrorMessage("Select row before remove");
            return;
        }
        mrvk_gui_interface::EraseWaypoint srv;
        srv.request.index = row;
        eraseWaypointSrv.call(srv);

        if (!srv.response.success) showErrorMessage(srv.response.message);
        updateWaypoints(srv.response.waypoints);
    }

    void WaypointsTableWidget::on_btnMoveUp_clicked() {
        int row = ui->tableWidget->currentRow();
        if (row < 0) {
            showErrorMessage("Select row before remove");
            return;
        }

        if (row - 1 < 0) {
            updateWaypoints();
            return;
        }

        mrvk_gui_interface::SwapWaypoints srv;
        srv.request.index1 = row;
        srv.request.index2 = row - 1;
        swapWaypointSrv.call(srv);

        if (!srv.response.success) showErrorMessage(srv.response.message);
        updateWaypoints(srv.response.waypoints, row -1);
    }

    void WaypointsTableWidget::on_btnMoveDown_clicked() {
        int row = ui->tableWidget->currentRow();
        if (row < 0) {
            showErrorMessage("Select row before remove");
            return;
        }

        if (row + 1 > ui->tableWidget->rowCount() - 1) {
            updateWaypoints();
            return;
        }

        mrvk_gui_interface::SwapWaypoints srv;
        srv.request.index1 = row;
        srv.request.index2 = row + 1;
        swapWaypointSrv.call(srv);

        if (!srv.response.success) showErrorMessage(srv.response.message);
        updateWaypoints(srv.response.waypoints, row + 1);
    }

    void WaypointsTableWidget::on_btnSyncTable_clicked() {
        updateWaypoints();
    }

    void WaypointsTableWidget::tableClicked(int row, int col) {
        if (col != 0) return;   // ignore all columns except first

        ROS_ERROR("%d, %d", row, col);
        QTableWidgetItem *item = ui->tableWidget->item(row, col);

        mrvk_gui_interface::EditWaypoint srv;
        srv.request.waypoint.latitude =  ui->tableWidget->item(row, 1)->text().toDouble();
        srv.request.waypoint.longitude = ui->tableWidget->item(row, 2)->text().toDouble();
        srv.request.index = row;

        if (item->checkState() == Qt::Checked) {
            qDebug("is checked");
            srv.request.waypoint.active = true;
        } else if (item->checkState() == Qt::Unchecked) {
            qDebug("is unchecked");
            srv.request.waypoint.active = false;
        }

        editWaypointSrv.call(srv);
        if (!srv.response.success) showErrorMessage(srv.response.message);
        updateWaypoints(srv.response.waypoints);
    }
}