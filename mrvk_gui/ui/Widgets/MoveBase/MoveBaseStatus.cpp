#include "MoveBaseStatus.h"
#include <ui_MoveBaseStatus.h>


namespace mrvk_gui {
    MoveBaseStatus::MoveBaseStatus(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBaseStatus),
            status("No goal active"),
            currentPos(0, 0, 0),
            goalPos(0, 0, 0) {
        ui->setupUi(this);
        ui->statusValue->setStyleSheet(QLABEL_COLOR_OK);
    }

    MoveBaseStatus::~MoveBaseStatus() {
        delete ui;
    }

    void MoveBaseStatus::activeCallback(){
        ui->statusValue->setStyleSheet(QLABEL_COLOR_OK);
        status = "Tracking goal.";
    }

    void MoveBaseStatus::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
        currentPos[0] = feedback->base_position.pose.position.x;
        currentPos[1] = feedback->base_position.pose.position.y;
    }

    void MoveBaseStatus::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
        using namespace actionlib;

        switch(state.state_) {
            case SimpleClientGoalState::ABORTED:
            case SimpleClientGoalState::REJECTED:
            case SimpleClientGoalState::PREEMPTED:
            case SimpleClientGoalState::LOST:
                ui->statusValue->setStyleSheet(QLABEL_COLOR_ERROR);
                break;

            case SimpleClientGoalState::PENDING:
            case SimpleClientGoalState::RECALLED:
                ui->statusValue->setStyleSheet(QLABEL_COLOR_WARNING);
                break;

            case SimpleClientGoalState::ACTIVE:
            case SimpleClientGoalState::SUCCEEDED:
                ui->statusValue->setStyleSheet(QLABEL_COLOR_OK);
                break;
        }
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