//
// Created by matejko on 15.8.2017.
//

#ifndef PROJECT_MRVKGUIPLUGIN_H
#define PROJECT_MRVKGUIPLUGIN_H
#include <rqt_gui_cpp/plugin.h>
#include <ui_MainWidget.h>
#include <mrvk_gui/DiagnosticsWidget.h>
#include <ui_ControlWidget.h>
#include <osm_planner/osm_parser.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


namespace mrvk_gui{

    class MrvkGui : public rqt_gui_cpp::Plugin {
    Q_OBJECT
    public:
        MrvkGui();
        virtual void initPlugin(qt_gui_cpp::PluginContext &context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    private:
        osm_planner::Parser::OSM_NODE map_origin, goal_target;
        geometry_msgs::PoseStamped goalXY;

        Ui::MainWidget mainUi;
        QWidget *mainWidget;
        Ui::ControlWidget controlWidget;
        //DiagnosticModel treeModel;
        ros::Publisher goal_pub, cancel_pub;
        ros::Subscriber result_sub, camera_sub;
        ros::ServiceClient init_robot;
        actionlib_msgs::GoalID cancel_goal_msg;

        virtual void listenResult(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);
        void listenCamera(const boost::shared_ptr<const sensor_msgs::Image_<std::allocator<void>>> &msg);

    private slots:
        void goToGoal();
        void cancelGoal();
        void readNavigData();

    };
};


#endif //PROJECT_MRVKGUIPLUGIN_H
