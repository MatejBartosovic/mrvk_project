#include "MrvkDriver.h"
#include <ui_MrvkDriver.h>
#include <QMessageBox>

namespace mrvk_gui {
    MrvkDriver::MrvkDriver(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MrvkDriver()) {
        ui->setupUi(this);
        connect(ui->setCentralStopButton,SIGNAL(released()),this,SLOT(setCentralStop()));
        connect(ui->resetCentralStopButton,SIGNAL(released()),this,SLOT(resetCentralStop()));
        connect(ui->blockMovementButton,SIGNAL(released()),this,SLOT(blockMovement()));
        connect(ui->unblockMovementButton,SIGNAL(released()),this,SLOT(unblockMovement()));

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
        catch (MrvkControllButtonsException& e){
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
        catch (MrvkControllButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Set central stop",string);
        msgBox.exec();
    }
    void MrvkDriver::blockMovement(){
        QString string;
        try{
            mrvkButtons.blockMovement();
            string = "Ok";
        }
        catch (MrvkControllButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Set central stop",string);
        msgBox.exec();
    }
    void MrvkDriver::unblockMovement(){
        QString string;
        try{
            mrvkButtons.unblockMovement();
            string = "Ok";
        }
        catch (MrvkControllButtonsException& e){
            string = e.what();
        }
        QMessageBox msgBox(QMessageBox::Information,"Set central stop",string);
        msgBox.exec();
    }

    void mrvk_gui::MrvkDriver::updateData() {
        ui->mainBoardStatus->updateData();
    }
}