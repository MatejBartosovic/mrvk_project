//
// Created by matejko on 15.8.2017.
//

#include "mrvk_gui/MrvkGuiPlugin.h"
#include <pluginlib/class_list_macros.h>
#include <QMessageBox>
#include <std_srvs/Trigger.h>
#include <osm_planner/coordinates_converters/haversine_formula.h>

namespace mrvk_gui {

    MrvkGui::MrvkGui() : rqt_gui_cpp::Plugin(), mainWidget(0)
    {
        coordinateConverter = std::make_shared<osm_planner::coordinates_converters::HaversineFormula>();

    }

    void MrvkGui::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        QStringList argv = context.argv();

        //setup mainWidget
        mainWidget = new QWidget();
        mainUi.setupUi(mainWidget);

        //setup diagnosticsWidget
        //diagnosticsWidget.setupUi(mainUi.tabWidget->widget(0));
        controlWidget.setupUi(mainUi.tabWidget->widget(1));

        //add to cintext
        context.addWidget(mainWidget);

        connect(controlWidget.setGoal,SIGNAL(clicked()),this,SLOT(setGoal()));
        connect(controlWidget.cancelGoal,SIGNAL(clicked()),this,SLOT(cancelGoal()));

        ros::NodeHandle n;

        //get map origin
        osm_planner::Parser parser;
        // from param server
        std::string filepath;
        std::vector<std::string> way_types;
        std::string global_frame;

        n.getParam("/move_base/Planner/osm_map_path",filepath);
        n.getParam("/move_base/Planner/global_frame",global_frame);
        n.getParam("/move_base/Planner/filter_of_ways",way_types);

        double latitude, longitude;
        int settingOrigin;
        n.param<double>("/move_base/Planner/origin_latitude", latitude,0.0);
        n.param<double>("/move_base/Planner/origin_longitude",longitude,0.0);
        n.param<int>("/move_base/Planner/set_origin_pose", settingOrigin,0);

        parser.setNewMap(filepath);
        parser.setTypeOfWays(way_types);

        if (settingOrigin == 3) {
            parser.parse();
           // map_origin = parser.getNodeByID(parser.getNearestPoint(latitude, longitude));
	        ROS_INFO("PRVA MOZNOST");
            map_origin.latitude = latitude;
            map_origin.longitude = longitude;

        }
        else{
            parser.parse(true);
            map_origin = parser.getNodeByID(0);
            // ROS_ERROR("lat %f, lon %f", mapOrigin.latitude,mapOrigin.longitude);
            ROS_INFO("DRUHA MOZNOST");
       
}

	    ROS_INFO("MAP ORIGIN LATITUDE %lf", map_origin.latitude );
	    ROS_INFO("MAP ORIGIN LONGITUDE %lf",map_origin.longitude );

        //Init goal message and publisher
        goalXY.header.frame_id = global_frame;
        goalXY.pose.position.x = 0;
        goalXY.pose.position.y = 0;
        goalXY.pose.position.z = 0;
        goalXY.pose.orientation.x = 0;
        goalXY.pose.orientation.y = 0;
        goalXY.pose.orientation.z = 0;
        goalXY.pose.orientation.w = 1;

        goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
        init_robot = n.serviceClient<std_srvs::Trigger>("/mrvk_supervisor/init");


    }

    void MrvkGui::shutdownPlugin()
    {
        // TODO unregister all publishers here
    }

    void MrvkGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void MrvkGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }

    void MrvkGui::readNavigData(){

        /*if(controlWidget.decimal_check){
            longitude.= controlWidget.long_stupne->toPlainText().toDouble();
        }*/

       // goal_target.longitude = ;

        //longitude
        double stupne = controlWidget.long_stupne->toPlainText().toDouble();
        double min = controlWidget.long_minuty->toPlainText().toDouble();
        double sec = controlWidget.long_sekundy->toPlainText().toDouble();
        goal_target.longitude = stupne + (min/60) + (sec/3600);

        //latitude
        stupne  = controlWidget.lat_stupne->toPlainText().toDouble();
        min  = controlWidget.lat_minuty->toPlainText().toDouble();
        sec  = controlWidget.lat_sekundy->toPlainText().toDouble();

        goal_target.latitude = stupne + (min/60) + (sec/3600);
        ROS_ERROR("MAP ORIGIN LATITUDE %lf",goal_target.latitude );
        ROS_ERROR("MAP ORIGIN LONGITUDE %lf", goal_target.longitude );

    }

    void MrvkGui::setGoal(){
        QMessageBox msgBox;
        msgBox.setText(QString("Do you want to start robot movement ?"));
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);

        std_srvs::Trigger trigger_init;

        switch (msgBox.exec()){
            case QMessageBox::Cancel:
                // Cancel was clicked
                break;
            case QMessageBox::Ok:
                this->readNavigData();
                if(init_robot.call(trigger_init) && trigger_init.response.success){
                    goalXY.pose.position.x = coordinateConverter->getCoordinateX(map_origin,goal_target);
                    goalXY.pose.position.y = coordinateConverter->getCoordinateY(map_origin,goal_target);
                    goalXY.header.stamp = ros::Time::now();
                    goal_pub.publish(goalXY);
                }
                else{
                    ROS_ERROR_STREAM("Problem with robot inicialization and goal setup");
                }


            default:
                // should never be reached
                break;
        }

    }

    void MrvkGui::cancelGoal(){
        QMessageBox msgBox;
        msgBox.setText(QString("Do you want to cancel robot movement ?"));
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);


        switch (msgBox.exec()){
            case QMessageBox::Cancel:
                // Cancel was clicked
                break;
            case QMessageBox::Ok:

                cancel_goal_msg.stamp = ros::Time::now();
                cancel_goal_msg.id="";
                cancel_pub.publish(cancel_goal_msg);

                break;
            default:
                // should never be reached
                break;
        }

    }


}; // namespace
PLUGINLIB_EXPORT_CLASS(mrvk_gui::MrvkGui, rqt_gui_cpp::Plugin)
