#ifndef MOVEBASESTATUS_H
#define MOVEBASESTATUS_H

#include <QWidget>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/console.h>
#include <rosconsole/macros_generated.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <Eigen/Core>
#include <mrvk_gui/GuiDefines.h>
#include <mrvk_gui_interface/PerformWaypointsAction.h>

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

        void feedbackCallback(const mrvk_gui_interface::PerformWaypointsFeedbackConstPtr& feedback);

        void doneCallback(const actionlib::SimpleClientGoalState& state, const mrvk_gui_interface::PerformWaypointsResultConstPtr& result);

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