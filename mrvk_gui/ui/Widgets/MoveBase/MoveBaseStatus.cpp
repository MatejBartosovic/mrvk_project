#include "MoveBaseStatus.h"
#include <ui_MoveBaseStatus.h>

namespace mrvk_gui {
    MoveBaseStatus::MoveBaseStatus(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBaseStatus),
            x(0),y(0){
        ui->setupUi(this);
    }

    MoveBaseStatus::~MoveBaseStatus() {
        delete ui;
    }

    void MoveBaseStatus::activeCallback(){
        status = "Tracking goal.";
    }

    void MoveBaseStatus::feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback){
        x = feedback->base_position.pose.position.x;
        y = feedback->base_position.pose.position.y;
    }

    void MoveBaseStatus::doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
        status = state.toString();
    }

    void MoveBaseStatus::updateData(){
        ui->goalState->setText(QString::fromStdString(status));
        ui->xLabel->setNum(x);
        ui->yLabel->setNum(y);
    }
}