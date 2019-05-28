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
        loadWaypointsFromFile(WAYPOINTS_TABLE_CONFIG_FILE);
    }

    WaypointsTableWidget::~WaypointsTableWidget() {
        saveWaypointsToFile(WAYPOINTS_TABLE_CONFIG_FILE);
        delete ui;
    }

    void WaypointsTableWidget::addWaypoint(bool active, const QString& latitude, const QString& longitude) {
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

        item = new QTableWidgetItem(latitude);
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        ui->tableWidget->setItem(row,1, item);

        item = new QTableWidgetItem(longitude);
        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
        ui->tableWidget->setItem(row,2, item);
    }

    void WaypointsTableWidget::editWaypoint(int row, const QString& latitude, const QString& longitude) {
        auto item = ui->tableWidget->item(row, 1);
        item->setText(latitude);
        item = ui->tableWidget->item(row, 2);
        item->setText(longitude);
    }


    void WaypointsTableWidget::saveWaypointsToFile(const QString& filepath) {
        QFile file(filepath);
        if (!file.open(QIODevice::WriteOnly)) {
            //error
            return;
        }

        QTextStream stream(&file);
        stream << "Active;Latitude;Longitude;" << endl;

        for (int i = 0; i < ui->tableWidget->rowCount(); i++) {
            auto item = ui->tableWidget->item(i, 0);
            if (item->checkState() == Qt::Checked) {
                stream << 1 << ";";
            } else {
                stream << 0 << ";";
            }

            item = ui->tableWidget->item(i, 1);
            stream << item->text() << ";";
            item = ui->tableWidget->item(i, 2);
            stream << item->text() << ";" << endl;
        }
    }

    void WaypointsTableWidget::loadWaypointsFromFile(const QString& filepath) {
        QFile file(filepath);
        if (!file.open(QIODevice::ReadOnly)) {
            //error
            return;
        }

        QTextStream stream(&file);
        QString line;
        line = stream.readLine();

        while (!stream.atEnd()) {
            line = stream.readLine();
            QStringList lineSplit = line.split(';');
            addWaypoint(lineSplit.at(0).toInt(), lineSplit.at(1), lineSplit.at(2));
        }
    }

    void WaypointsTableWidget::on_btnAddNew_clicked() {
        mrvk_gui::AddWaypointDialog addNew(this);
        int dialogCode = addNew.exec();
        if (dialogCode == 0) {
            mrvk_gui::QGeoPose geoPose = addNew.getGeoPose();
            qDebug(geoPose.latitude.toLatin1() + ", " + geoPose.longitude.toLatin1());
            addWaypoint(true, geoPose.latitude, geoPose.longitude);
            saveWaypointsToFile(WAYPOINTS_TABLE_CONFIG_FILE);
        } else {
            qDebug("Cancel");
        }
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
            showErrorMessage("Select row before");
            return;
        }

        mrvk_gui::AddWaypointDialog addNew(this);

        QString latitude = ui->tableWidget->item(row, 1)->text();
        QString longitude = ui->tableWidget->item(row, 2)->text();

        int dialogCode = addNew.execInitGeoPose(latitude, longitude);
        if (dialogCode == 0) {
            mrvk_gui::QGeoPose geoPose = addNew.getGeoPose();
            if (geoPose.latitude.isEmpty() || geoPose.longitude.isEmpty()) {
                return;
            }
            editWaypoint(row, geoPose.latitude, geoPose.longitude);
            saveWaypointsToFile(WAYPOINTS_TABLE_CONFIG_FILE);
        } else {
            qDebug("Cancel");
        }

    }

    void WaypointsTableWidget::on_btnAddCurrent_clicked() {
        auto data = subscriber->getData();
        addWaypoint(true, QString::number(data.latitude), QString::number(data.longitude));
    }

    void WaypointsTableWidget::showErrorMessage(const QString& message){
        QMessageBox msgBox(QMessageBox::Critical, "Error", message);
        msgBox.exec();
    }

    std::list<Waypoint> WaypointsTableWidget::getWaypoints() {
        std::list<Waypoint> way;
        QTableWidgetItem *item;

        for (int i = 0; i < ui->tableWidget->rowCount(); i++) {
            item = ui->tableWidget->item(i, 0);
            if (item->checkState() == Qt::Checked) {
                Waypoint point;
                //TODO body treba prekonvertovat z Geo na Map
                point.x = ui->tableWidget->item(i, 1)->text().toDouble();
                point.y = ui->tableWidget->item(i, 2)->text().toDouble();
                way.push_back(point);
            }
        }

        return way;
    }

}