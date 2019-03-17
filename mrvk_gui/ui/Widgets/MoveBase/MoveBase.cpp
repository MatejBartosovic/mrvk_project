#include "MoveBase.h"
#include <ui_MoveBase.h>
#include <ui_MoveBaseControl.h>
#include <eigen_conversions/eigen_msg.h>
#include <QMessageBox>

namespace mrvk_gui {
    MoveBase::MoveBase(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBase),
            actionClient("move_base", true){
        ui->setupUi(this);

        connect(ui->moveBaseControl->ui->goButton,SIGNAL(released()),this,SLOT(goSlot()));
        connect(ui->moveBaseControl->ui->cancelButton,SIGNAL(released()),this,SLOT(cancelSlot()));
        connect(ui->moveBaseControl->ui->mapOffsetButton,SIGNAL(released()),this,SLOT(editMapOffsetSlot()));
        connect(ui->moveBaseControl->ui->loadQrCodeButton,SIGNAL(released()),this,SLOT(readQrCodeSlot()));
    }

    MoveBase::~MoveBase() {
        delete ui;
    }

    void MoveBase::updateData(){
        ui->moveBaseStatus->updateData();
        ui->moveBaseControl->updateData();
    }

    void MoveBase::goSlot(){
        if(!actionClient.isServerConnected() &&  !actionClient.waitForServer(ros::Duration(0.5))){
            QMessageBox box(QMessageBox::Icon::Information,"Action client","Action server is not running. Abborting goal.",QMessageBox::Button::Ok,this);
            box.exec();
            return;
        }
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.header.frame_id = "map"; //TODO parameter alebo combo box zo vsetkimi framami??
        tf::quaternionEigenToMsg(Eigen::Quaterniond(1,0,0,0),goal.target_pose.pose.orientation);
        goal.target_pose.pose.position.x = 0; //TODO
        goal.target_pose.pose.position.y = 0; //TODO
        goal.target_pose.pose.position.z = 0;

        actionClient.sendGoal(goal,boost::bind(&MoveBaseStatus::doneCallback, ui->moveBaseStatus, _1, _2),
                boost::bind(&MoveBaseStatus::activeCallback, ui->moveBaseStatus),
                boost::bind(&MoveBaseStatus::feedbackCallback, ui->moveBaseStatus, _1));
    }

    void MoveBase::cancelSlot(){
        actionClient.cancelGoal();
    }

    void MoveBase::editMapOffsetSlot(){
        QMessageBox box(QMessageBox::Icon::Information,"Feature not implemented","Feature not implemented",QMessageBox::Button::Ok,this);
        box.exec();
    }

    void MoveBase::readQrCodeSlot(){
        QMessageBox box(QMessageBox::Icon::Information,"Feature not implemented","Feature not implemented",QMessageBox::Button::Ok,this);
        box.exec();
    }
}