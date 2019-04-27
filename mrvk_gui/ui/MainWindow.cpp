#include "MainWindow.h"
#include <ui_MainWindow.h>
#include <iostream>

namespace mrvk_gui {
    MainWindow::MainWindow(QWidget* parent) :
            QMainWindow(parent),
            ui(new Ui::MainWindow),
            timer(this){
        ui->setupUi(this);
        connect(&timer,SIGNAL(timeout()),this,SLOT(updateGui()));
        timer.start(1000);
    }

    MainWindow::~MainWindow() {
        delete ui;
    }

    void MainWindow::updateGui(){
        ui->topicFrequency->updateData();
        ui->odometry->updateData();
        ui->imu->updateData();
        ui->odometry->updateData();
        ui->mrvkDriver->updateData();
        ui->gps->updateData();
        ui->camera->updateData();
        ui->moveBase->updateData();
    }
}