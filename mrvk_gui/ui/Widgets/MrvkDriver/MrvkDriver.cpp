#include "MrvkDriver.h"
#include <ui_MrvkDriver.h>
#include <QMessageBox>
#include <mrvk_gui/ButtonException.h>

namespace mrvk_gui {
    MrvkDriver::MrvkDriver(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MrvkDriver()) {
        ui->setupUi(this);
        connect(ui->setCentralStopButton,SIGNAL(released()),this,SLOT(setCentralStop()));
        connect(ui->resetCentralStopButton,SIGNAL(released()),this,SLOT(resetCentralStop()));
        connect(ui->blockMovementButton,SIGNAL(released()),this,SLOT(blockMovement()));
        connect(ui->unblockMovementButton,SIGNAL(released()),this,SLOT(unblockMovement()));
        connect(ui->autoComputeGPSButton,SIGNAL(released()),this,SLOT(autoComputeGPS()));
        connect(ui->drawRoadsButton,SIGNAL(released()),this,SLOT(drawRoads()));
    }

    MrvkDriver::~MrvkDriver() {
        delete ui;
    }

    void MrvkDriver::setCentralStop(){
        QString string;
        try{
            mrvkButtons.setCentralStop();
            string = "Ok";
        }
        catch (ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Set central stop",string);
        msgBox.exec();
    }
    void MrvkDriver::resetCentralStop(){
        QString string;
        try{
            mrvkButtons.resetCentralStop();
            string = "Ok";
        }
        catch (ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Reset central stop",string);
        msgBox.exec();
    }
    void MrvkDriver::blockMovement(){
        QString string;
        try{
            mrvkButtons.blockMovement();
            string = "Ok";
        }
        catch (ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Block movement",string);
        msgBox.exec();
    }
    void MrvkDriver::unblockMovement(){
        QString string;
        try{
            mrvkButtons.unblockMovement();
            string = "Ok";
        }
        catch (ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Unblock movement",string);
        msgBox.exec();
    }
    void MrvkDriver::autoComputeGPS() {
        QString string;
        try{
            double angle = mrvkButtons.autoComputeGPS();
            string = QString("Ok. Angle: ") + QString::number(angle);
        }
        catch(ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Auto compute GPS",string);
        msgBox.exec();
    }
    void mrvk_gui::MrvkDriver::updateData() {
        ui->mainBoardStatus->updateData();
    }

    void mrvk_gui::MrvkDriver::drawRoads(){
        QString string;
        try{
            mrvkButtons.drawRoads();
            string = "Ok";
        }
        catch(ButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Auto compute GPS",string);
        msgBox.exec();
    }
}