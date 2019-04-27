#ifndef MOVEBASE_H
#define MOVEBASE_H

#include <QWidget>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <eigen_conversions/eigen_msg.h>
#include <QMessageBox>
#include <osm_planner/coordinates_converters/haversine_formula.h>
#include <string>
#include "GpsCoordinatesInput.h"
#include <fstream>
#include <ros/ros.h>
#include <sstream>
#include <tf/transform_listener.h>
#include <mrvk_gui/GuiDefines.h>
#include <QThread>
#include <atomic>

namespace Ui {
class MoveBase;
}

namespace mrvk_gui {
    class MoveBase : public QWidget {
    Q_OBJECT

    public:
        explicit MoveBase(QWidget* parent = 0);

        ~MoveBase();

        void updateData();

    private:
//        void loadDefaultMapOffset();
//        void saveDefaultMapOffset();
        Ui::MoveBase* ui;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> actionClient;
        double latitudeMapOffset = 0;
        double longitudeMapOffset = 0;
        bool setGoalByOffset = false;
        QThread goalThread;

        void doneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
        std::atomic<bool> goalDone;

    public slots:
        void goSlot();
        void cancelSlot();
        void editMapOffsetSlot();
        void readQrCodeSlot();
        void goByOffsetCbx(bool value);
        void trackGoals();

    };
}
#endif // MOVEBASE_H
