//
// Created by matejko on 15.8.2017.
//

#include "mrvk_gui/MrvkGuiPlugin.h"

namespace mrvk_gui {

    MrvkGui::MrvkGui() : rqt_gui_cpp::Plugin(), mainWidget(0){

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

        // connect slots for buttons
        connect(controlWidget.goToGoal_btn, SIGNAL(clicked()), this, SLOT(goToGoal_btn()));
        connect(controlWidget.cancelGoal_btn, SIGNAL(clicked()), this, SLOT(cancelGoal_btn()));
        connect(controlWidget.scanQrStart_btn, SIGNAL(clicked()), this, SLOT(scanQrStart_btn()));
        connect(controlWidget.scanQrStop_btn, SIGNAL(clicked()), this, SLOT(scanQrStop_btn()));
        connect(controlWidget.storePosition_btn, SIGNAL(clicked()), this, SLOT(storeActualPosition_btn()));
        connect(controlWidget.restorePosition_btn, SIGNAL(clicked()), this, SLOT(restorePosition_btn()));

        // slot for update gui data
        connect(this, SIGNAL(gpsValueChanged(double, double)), this, SLOT(updateGuiGPS(double, double)));
        connect(this, SIGNAL(diagnosticDataChanged(QString, QString, QString)),
                this, SLOT(updateGuiDiagnostic(QString, QString, QString)));

        // slot for ventilator control
        connect(controlWidget.ventilator_cbx, SIGNAL(clicked(bool)), this, SLOT(ventilator_cbx()));

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
        init_robot = n.serviceClient<std_srvs::SetBool>("/mrvk_supervisor/init");

        result_sub = n.subscribe("/move_base/result", 5, &MrvkGui::listenResult,this);
        diagnostic_sub = n.subscribe("/diagnostics",1, &MrvkGui::listenDiagnosticMsg,this);

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
        /*
         * Read data from entry forms as the target Goal
         */

        // longitude
        double stupne = controlWidget.long_stupne->toPlainText().toDouble();
        double min = controlWidget.long_minuty->toPlainText().toDouble();
        double sec = controlWidget.long_sekundy->toPlainText().toDouble();
        goal_target.longitude = stupne + (min/60) + (sec/3600);

        // latitude
        stupne  = controlWidget.lat_stupne->toPlainText().toDouble();
        min  = controlWidget.lat_minuty->toPlainText().toDouble();
        sec  = controlWidget.lat_sekundy->toPlainText().toDouble();
        goal_target.latitude = stupne + (min/60) + (sec/3600);

        // TODO log this data

        /*ROS_ERROR("MAP ORIGIN LATITUDE %lf",goal_target.latitude );
        ROS_ERROR("MAP ORIGIN LONGITUDE %lf", goal_target.longitude );*/
    }

    void MrvkGui::goToGoal_btn(){
        QMessageBox msgBox;
        msgBox.setText(QString("Do you want to start robot movement ?"));
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);

        std_srvs::SetBool setbool_init;

        setbool_init.request.data = (unsigned char) controlWidget.sever->isTristate();

        switch (msgBox.exec()){
            case QMessageBox::Cancel:
                // Cancel was clicked
                break;
            case QMessageBox::Ok:
                this->readNavigData();

                controlWidget.information_label->setText("CALLED INIT ROBOT (gyro calibration)");
                // this service "init_robot" calibrate gyro and gps
                if (init_robot.call(setbool_init) && setbool_init.response.success) {
                        goalXY.pose.position.x = osm_planner::Parser::Haversine::getCoordinateX(map_origin,
                                                                                                goal_target);
                        goalXY.pose.position.y = osm_planner::Parser::Haversine::getCoordinateY(map_origin,
                                                                                                goal_target);
                        goalXY.header.stamp = ros::Time::now();
                        goal_pub.publish(goalXY);
                        controlWidget.information_label->setText("GOING TO GOAL");
                } else {
                        ROS_ERROR_STREAM("Problem with robot inicialization and goal setup");
                        controlWidget.information_label->setText("ERROR OCCURED WHILE INIT ROBOT");
                }
            default:
                // should never be reached
                break;
        }
    }

    void MrvkGui::cancelGoal_btn() {
        QMessageBox msgBox;
        msgBox.setText(QString("Do you want to cancel robot movement ?"));
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);

        switch (msgBox.exec()) {
            case QMessageBox::Cancel:
                // Cancel was clicked
                break;
            case QMessageBox::Ok:

                cancel_goal_msg.stamp = ros::Time::now();
                cancel_goal_msg.id = "";
                cancel_pub.publish(cancel_goal_msg);

                break;
            default:
                // should never be reached
                break;
        }
    }

    void MrvkGui::listenResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
        /*
         * Some result is received
         *
         * msg->status;
         *  PENDING=0
         *  ACTIVE=1
         *  PREEMPTED=2
         *  SUCCEEDED=3
         *  ABORTED=4
         *  REJECTED=5
         *  PREEMPTING=6
         *  RECALLING=7
         *  RECALLED=8         *
         *
         */

        if (msg->status.status == actionlib_msgs::GoalStatus::SUCCEEDED){
            controlWidget.information_label->setText("THE GOAL HAS BEEN REACHED");

        } else if (msg->status.status == actionlib_msgs::GoalStatus::ABORTED){
            controlWidget.information_label->setText("THE GOAL HAS BEEN ABORTED");

        } else {
            controlWidget.information_label->setText("ERR: GOAL HASN'T BEED REACHED");
        }
    }

    void MrvkGui::listenCamera(const sensor_msgs::ImageConstPtr& msg){
        try{
            cv_bridge::CvImageConstPtr cvb_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat mat = cvb_img->image;
            auto pixmap = QPixmap::fromImage(QImage((unsigned char*) mat.data, mat.cols, mat.rows, QImage::Format_RGB888));
            controlWidget.cameraOutput->setPixmap(pixmap);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'RGB8'.", msg->encoding.c_str());
        }
    }

    void MrvkGui::listenQrData(const std_msgs::String &msg){
        //"geo:48.15312,17.07437"
        using namespace std;

        string geo = msg.data;
        ROS_INFO("QR Data: %s", msg.data.c_str());

        if (geo.size() < 4) {
            ROS_ERROR("Read code have less then 4 chars: %s", msg.data.c_str());
            return;
        }
        geo.erase(geo.begin(), geo.begin() + 4);
        stringstream ss(geo);

        double latitude = 0, longitude = 0;
        char delimiter;
        ss>>longitude;
        ss>>delimiter;
        ss>>latitude;

        if (latitude == 0 || longitude == 0){
            ROS_ERROR("QR code is damaged: %s", msg.data.c_str());
            return;
        }

        ROS_INFO("Longitude = %f", longitude);
        ROS_INFO("Latitude   = %f", latitude);

        // change values in UI forms
        emit gpsValueChanged(longitude, latitude);

        // shutdown unnecessary subscribers
        camera_sub.shutdown();
        qr_data_sub.shutdown();

        controlWidget.information_label->setText("QR RECOGNIZED");
    }

    void MrvkGui::updateGuiGPS(double longitude, double latitude) {
        // latitude
        controlWidget.lat_stupne->setText(QString::number(latitude, 'f', 7));
        controlWidget.lat_minuty->setText("0");
        controlWidget.lat_sekundy->setText("0");

        // longitude
        controlWidget.long_stupne->setText(QString::number(longitude, 'f', 7));
        controlWidget.long_minuty->setText("0");
        controlWidget.long_sekundy->setText("0");

        controlWidget.information_label->setText("GPS FORMS UPDATED");
    }

    void MrvkGui::scanQrStart_btn() {
        controlWidget.information_label->setText("SCANNING QR");
        camera_sub = n.subscribe("/usb_cam/image_raw", 1, &MrvkGui::listenCamera,this);
        qr_data_sub = n.subscribe("/qr_detector/qr_codes", 1, &MrvkGui::listenQrData,this);
    }

    void MrvkGui::scanQrStop_btn() {
        controlWidget.information_label->setText("SCANNING STOPPED");
        // shutdown subscribers
        camera_sub.shutdown();
        qr_data_sub.shutdown();
    }

    void MrvkGui::storeActualPosition_btn() {
        gps_fix_sub = n.subscribe("odom", 1, &MrvkGui::listenGpsFix, this);
    }

    void MrvkGui::listenGpsFix(const sensor_msgs::NavSatFixConstPtr &msg) {
        storedPosition.clear();
        storedPosition.push_back(msg->longitude);
        storedPosition.push_back(msg->latitude);
        gps_fix_sub.shutdown();

        controlWidget.information_label->setText("POSITION STORED");
        ROS_INFO("Longitude = %f", msg->longitude);
        ROS_INFO("Latitude   = %f", msg->latitude);
    }

    void MrvkGui::restorePosition_btn() {
        if(storedPosition.empty()) {
            controlWidget.information_label->setText("ANY POSITION STORED");
            return;
        }

        // change values in UI forms
        emit gpsValueChanged(storedPosition.at(0), storedPosition.at(1));
        storedPosition.clear();
    }

    void MrvkGui::listenDiagnosticMsg(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {
        using namespace std;
        // check if diagnostic mesage is correct
        bool correct_diag_msg = false;
        for(auto status:msg->status){
            if(status.name == "mrvk_driver: mrvk_driver Status"){
                correct_diag_msg = true;
            }
        }
        if(!correct_diag_msg){
            return;
        }

        // get the diagnostic value
        string value;
        for(auto values:msg->status[0].values) {
            if(values.key == "main board status") {
                value = values.value;
                break;
            }
        }

        // parse target data
        string key_word;
        size_t begin, end;
        key_word = "battery1_voltage: ";
        begin = value.find(key_word) + key_word.length();
        end = value.find('\n', value.find(key_word) + 1);
        string battery1 = value.substr(begin, end - begin); //get value behind key_word word

        key_word = "battery2_voltage: ";
        begin = value.find(key_word) + key_word.length();
        end = value.find('\n', value.find(key_word) + 1);
        string battery2 = value.substr(begin, end - begin);

        key_word = "current: ";
        begin = value.find(key_word) + key_word.length();
        end = value.find('\n', value.find(key_word) + 1);
        string current = value.substr(begin, end - begin);

        // update gui
        emit diagnosticDataChanged(QString::fromStdString(battery1),
                                   QString::fromStdString(battery2),
                                   QString::fromStdString(current));

    }

    void MrvkGui::updateGuiDiagnostic(QString battery1, QString battery2, QString current) {
        controlWidget.battery1_label->setText(battery1);
        controlWidget.battery2_label->setText(battery2);
        controlWidget.current_label->setText(current);
    }

    void MrvkGui::ventilator_cbx() {
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::BoolParameter bool_param;
        dynamic_reconfigure::Config conf;

        bool_param.name = "wifi";
        bool_param.value = controlWidget.ventilator_cbx->isChecked();

        conf.bools.push_back(bool_param);
        srv_req.config = conf;

        ros::service::call("/mrvk_driver/set_parameters", srv_req, srv_resp);
    }


}; // namespace
PLUGINLIB_EXPORT_CLASS(mrvk_gui::MrvkGui, rqt_gui_cpp::Plugin)

