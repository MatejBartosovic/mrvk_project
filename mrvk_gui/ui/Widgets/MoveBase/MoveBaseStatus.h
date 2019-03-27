#ifndef MOVEBASESTATUS_H
#define MOVEBASESTATUS_H

#include <QWidget>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <Eigen/Core>

namespace Ui {
class MoveBaseStatus;
}

namespace mrvk_gui {
    class MoveBaseStatus : public QWidget {
    Q_OBJECT

    public:
        explicit MoveBaseStatus(QWidget* parent = 0);

        ~MoveBaseStatus();

        void activeCallback();

        void feedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

        void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

        void updateData();

        void setGoal(double x, double y);
    private:
        Ui::MoveBaseStatus* ui;
        Eigen::Vector3d currentPos;
        Eigen::Vector3d goalPos;
        std::string status;
    };
}
#endif // MOVEBASESTATUS_H