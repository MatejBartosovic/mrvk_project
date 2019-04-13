#include "MoveBaseStatus.h"
#include <ui_MoveBaseStatus.h>

namespace mrvk_gui {
    MoveBaseStatus::MoveBaseStatus(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBaseStatus),
           status("No goal active"),
           currentPos(0,0,0),
           goalPos(0,0,0){
        ui->setupUi(this);
    }

    MoveBaseStatus::~MoveBaseStatus() {
        delete ui;
    }

    void MoveBaseStatus::activeCallback(){
        status = "Tracking goal.";
    }

    void MoveBaseStatus::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
        currentPos[0] = feedback->base_position.pose.position.x;
        currentPos[1] = feedback->base_position.pose.position.y;
    }

    void MoveBaseStatus::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
        status = state.toString();
    }

    void MoveBaseStatus::updateData(){
        ui->statusValue->setText(QString::fromStdString(status));
        ui->currentXvalue->setText(QString::number(currentPos[0], 'f', 4));
        ui->currentYValue->setText(QString::number(currentPos[1], 'f', 4));
        ui->remainingValue->setText(QString::number((currentPos - goalPos).norm(), 'f', 4));
    }
    void MoveBaseStatus::setGoal(double x, double y){
        goalPos[0] = x;
        goalPos[1] = y;
        ui->goalXValue->setText(QString::number(x, 'f', 4));
        ui->goalYValue->setText(QString::number(y, 'f', 4));
    }
}