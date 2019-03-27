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
        ui->currentXvalue->setNum(currentPos[0]);
        ui->currentYValue->setNum(currentPos[1]);
        ui->remainingValue->setNum((currentPos - goalPos).norm());
    }
    void MoveBaseStatus::setGoal(double x, double y){
        goalPos[0] = x;
        goalPos[1] = y;
        ui->goalXValue->setNum(x);
        ui->goalYValue->setNum(y);
    }
}