//
// Created by matejko on 15.8.2017.
//

#ifndef PROJECT_MRVKGUIPLUGIN_H
#define PROJECT_MRVKGUIPLUGIN_H

// ui
#include <rqt_gui_cpp/plugin.h>
#include <ui_MainWidget.h>
#include <mrvk_gui/DiagnosticsWidget.h>
#include <ui_ControlWidget.h>
#include <QMessageBox>
#include <pluginlib/class_list_macros.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <sensor_msgs/NavSatFix.h>

// image transport
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// others
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <osm_planner/osm_parser.h>
#include <vector>

// dinamic reconfigure
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


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
        ros::NodeHandle n;
        ros::Publisher goal_pub, cancel_pub;
        ros::Subscriber result_sub, camera_sub, qr_data_sub, diagnostic_sub, gps_fix_sub;
        ros::ServiceClient init_robot;
        actionlib_msgs::GoalID cancel_goal_msg;
        std::vector<double> storedPosition;     // memory for start position

        virtual void listenResult(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg);
        void listenCamera(const sensor_msgs::ImageConstPtr &msg);
        void listenQrData(const std_msgs::String &msg);
        void listenDiagnosticMsg(const diagnostic_msgs::DiagnosticArrayConstPtr &msg);
        void listenGpsFix(const sensor_msgs::NavSatFixConstPtr &msg);
        void readNavigData();

    signals:
        void gpsValueChanged(double longitude, double latitude);
        void diagnosticDataChanged(QString battery1, QString battery2, QString current);

    private slots:
        void goToGoal_btn();
        void cancelGoal_btn();
        void scanQrStart_btn();
        void scanQrStop_btn();
        void storeActualPosition_btn();
        void restorePosition_btn();
        void updateGuiGPS(double latitude, double longitude);
        void updateGuiDiagnostic(QString battery1, QString battery2, QString current);
        void ventilator_cbx();

    };
}


#endif //PROJECT_MRVKGUIPLUGIN_H
