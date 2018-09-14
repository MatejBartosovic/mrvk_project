//
// Created by matejko on 8.9.2018.
//

#include <mrvk_gui/RobotourControl.h>

#include <QFileDialog>
#include <QMessageBox>
#include <QIntValidator>

#include <ros/console.h>

RobotourControl::RobotourControl(){

}


void RobotourControl::setupUi(QWidget *parrent){
    Ui::ControlWidget::setupUi(parrent);

    Ui::ControlWidget::long_stupne->setValidator(new QIntValidator(0,60));
    Ui::ControlWidget::long_minuty->setValidator(new QIntValidator(0,60));
    Ui::ControlWidget::long_sekundy->setValidator(new QIntValidator(180,-180));
    Ui::ControlWidget::lat_stupne->setValidator(new QIntValidator(90,-90));
    Ui::ControlWidget::lat_minuty->setValidator(new QIntValidator(0,60));
    Ui::ControlWidget::lat_sekundy->setValidator(new QIntValidator(0,60));

    connect(Ui::ControlWidget::autoScroll,SIGNAL(stateChanged(int)), this, SLOT(autoscroll(int)));
    Ui::ControlWidget::autoScroll->setChecked(true);

    // connect slots for buttons
    connect(Ui::ControlWidget::goToGoal_btn, SIGNAL(clicked()), this, SLOT(goToGoal_btn()));
    connect(Ui::ControlWidget::cancelGoal_btn, SIGNAL(clicked()), this, SLOT(cancelGoal_btn()));
    connect(Ui::ControlWidget::scanQrStart_btn, SIGNAL(clicked()), this, SLOT(scanQrStart_btn()));
    connect(Ui::ControlWidget::scanQrStop_btn, SIGNAL(clicked()), this, SLOT(scanQrStop_btn()));
    connect(Ui::ControlWidget::storePosition_btn, SIGNAL(clicked()), this, SLOT(storeActualPosition_btn()));
    connect(Ui::ControlWidget::restorePosition_btn, SIGNAL(clicked()), this, SLOT(restorePosition_btn()));
    connect(Ui::ControlWidget::BrowseBtn, SIGNAL(clicked()), this, SLOT(setMap()));
    connect(Ui::ControlWidget::init_btn, SIGNAL(clicked()), this, SLOT(init_btm()));


    // slot for update gui data
    connect(this, SIGNAL(gpsValueChanged(double, double)), this, SLOT(updateGuiGPS(double, double)));
    connect(this, SIGNAL(diagnosticDataChanged(QString, QString, QString)),this, SLOT(updateGuiDiagnostic(QString, QString, QString)));

    // slot for ventilator control
    connect(Ui::ControlWidget::ventilator_cbx, SIGNAL(clicked(bool)), this, SLOT(ventilator_cbx()));

    // from param server
    std::string filepath;
    std::vector<std::string> way_types;
    std::string global_frame;

    n.getParam("/move_base/Planner/global_frame",global_frame);
    n.getParam("/move_base/Planner/filter_of_ways",way_types);

    double latitude, longitude;
    int settingOrigin;
    n.param<double>("/move_base/Planner/origin_latitude", latitude,0.0);
    n.param<double>("/move_base/Planner/origin_longitude",longitude,0.0);
    n.param<int>("/move_base/Planner/set_origin_pose", settingOrigin,0);

    //if (settingOrigin == 3) {
        // map_origin = parser.getNodeByID(parser.getNearestPoint(latitude, longitude));
        ROS_INFO("PRVA MOZNOST");
        map_origin.latitude = latitude;
        map_origin.longitude = longitude;

    //}


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

    result_sub = n.subscribe("/move_base/result", 5, &RobotourControl::listenResult,this);
    diagnostic_sub = n.subscribe("/diagnostics",1, &RobotourControl::listenDiagnosticMsg,this);

    has_qr = false;
}
void RobotourControl::readNavigData(){
    /*
     * Read data from entry forms as the target Goal
     */

    // longitude
    double stupne = Ui::ControlWidget::long_stupne->text().toDouble();
    double min = Ui::ControlWidget::long_minuty->text().toDouble();
    double sec = Ui::ControlWidget::long_sekundy->text().toDouble();
    goal_target.longitude = stupne + (min/60) + (sec/3600);

    // latitude
    stupne  = Ui::ControlWidget::lat_stupne->text().toDouble();
    min  = Ui::ControlWidget::lat_minuty->text().toDouble();
    sec  = Ui::ControlWidget::lat_sekundy->text().toDouble();
    goal_target.latitude = stupne + (min/60) + (sec/3600);

    // TODO log this data

    /*ROS_ERROR("MAP ORIGIN LATITUDE %lf",goal_target.latitude );
    ROS_ERROR("MAP ORIGIN LONGITUDE %lf", goal_target.longitude );*/
}

void RobotourControl::goToGoal_btn(){
    QMessageBox msgBox(QMessageBox::Question,"","Do you want to start robot movement ?",QMessageBox::Ok | QMessageBox::Cancel);

    std_srvs::SetBool setbool_init;

    setbool_init.request.data = (unsigned char) Ui::ControlWidget::sever->isTristate();

    switch (msgBox.exec()){
        case QMessageBox::Cancel:
            // Cancel was clicked
            break;
        case QMessageBox::Ok:
            this->readNavigData();
            //Ui::ControlWidget::information_wiev->addItem("CALLED INIT ROBOT (gyro calibration)");
		    //ROS_ERROR("Lat %l", goal_target.latitude);
		    //ROS_ERROR("Lon %l", goal_target.longitude);
		    //std::cout << "Goal latitude " << goal_target.latitude << std::endl;
		    //std::cout << "Goal longtitude " << goal_target.longtitude << std::endl;
		    Ui::ControlWidget::information_wiev->addItem("Goal Longtitude Latitude");
            Ui::ControlWidget::information_wiev->addItem(QString::number(goal_target.latitude, 'f', 9));
            Ui::ControlWidget::information_wiev->addItem(QString::number(goal_target.longitude, 'f', 9));
		    Ui::ControlWidget::information_wiev->addItem("Map origin Longitude Latitude");
		    Ui::ControlWidget::information_wiev->addItem(QString::number(map_origin.longitude, 'f', 9));
            Ui::ControlWidget::information_wiev->addItem(QString::number(map_origin.latitude, 'f', 9));
            Ui::ControlWidget::information_wiev->addItem("CALLED INIT ROBOT (gyro calibration)");
            // this service "init_robot" calibrate gyro and gps
//            if (init_robot.call(setbool_init) && setbool_init.response.success) {
//                goalXY.pose.position.x = osm_planner::Parser::Haversine::getCoordinateX(map_origin,
//                                                                                        goal_target);
//                goalXY.pose.position.y = osm_planner::Parser::Haversine::getCoordinateY(map_origin,
//                                                                                        goal_target);
//                goalXY.header.stamp = ros::Time::now();
//                goal_pub.publish(goalXY);
//                Ui::ControlWidget::information_wiev->addItem("GOING TO GOAL");
//            } else {
//                ROS_ERROR_STREAM("Problem with robot inicialization and goal setup");
//                Ui::ControlWidget::information_wiev->addItem("ERROR OCCURED WHILE INIT ROBOT");
//            }
            goalXY.pose.position.x = osm_planner::Parser::Haversine::getCoordinateX(map_origin,
                                                                                    goal_target);
            goalXY.pose.position.y = osm_planner::Parser::Haversine::getCoordinateY(map_origin,
                                                                                    goal_target);
            goalXY.header.stamp = ros::Time::now();
            goal_pub.publish(goalXY);
        default:
            // should never be reached
            break;
    }
}

void RobotourControl::cancelGoal_btn() {
    QMessageBox msgBox(QMessageBox::Question,"","Do you want to start robot movement ?",QMessageBox::Ok | QMessageBox::Cancel);


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

void RobotourControl::listenResult(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
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
        Ui::ControlWidget::information_wiev->addItem("THE GOAL HAS BEEN REACHED");


//        scanQrStart_btn();
//        while (!has_qr){
//           // sleep(1);
//            ros::spinOnce();
//        }
//        has_qr = false;
//        goToGoal_btn();

    } else if (msg->status.status == actionlib_msgs::GoalStatus::ABORTED){
        Ui::ControlWidget::information_wiev->addItem("THE GOAL HAS BEEN ABORTED");

    } else {
        Ui::ControlWidget::information_wiev->addItem("ERR: GOAL HASN'T BEED REACHED");
    }

    scanQrStart_btn();
}

void RobotourControl::listenCamera(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_bridge::CvImageConstPtr cvb_img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat mat = cvb_img->image;
        auto pixmap = QPixmap::fromImage(QImage((unsigned char*) mat.data, mat.cols, mat.rows, QImage::Format_RGB888));
        Ui::ControlWidget::cameraOutput->setPixmap(pixmap);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'RGB8'.", msg->encoding.c_str());
    }
}

void RobotourControl::listenQrData(const std_msgs::String &msg){
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

    ss>>latitude;
    ss>>delimiter;
    ss>>longitude;

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

   // has_qr = true;
    Ui::ControlWidget::information_wiev->addItem("QR RECOGNIZED");
    goToGoal_btn();
}

void RobotourControl::updateGuiGPS(double longitude, double latitude) {
    // latitude
    Ui::ControlWidget::lat_stupne->setText(QString::number(latitude, 'f', 7));
    Ui::ControlWidget::lat_minuty->setText("0");
    Ui::ControlWidget::lat_sekundy->setText("0");

    // longitude
    Ui::ControlWidget::long_stupne->setText(QString::number(longitude, 'f', 7));
    Ui::ControlWidget::long_minuty->setText("0");
    Ui::ControlWidget::long_sekundy->setText("0");

    Ui::ControlWidget::information_wiev->addItem("GPS FORMS UPDATED");
}

void RobotourControl::scanQrStart_btn() {
    Ui::ControlWidget::information_wiev->addItem("SCANNING QR");
    camera_sub = n.subscribe("/usb_cam/image_raw", 1, &RobotourControl::listenCamera,this);
    qr_data_sub = n.subscribe("/qr_detector/qr_codes", 1, &RobotourControl::listenQrData,this);
}

void RobotourControl::scanQrStop_btn() {
    Ui::ControlWidget::information_wiev->addItem("SCANNING STOPPED");
    // shutdown subscribers
    camera_sub.shutdown();
    qr_data_sub.shutdown();
}

void RobotourControl::storeActualPosition_btn() {
    storedPosition.clear();
    gps_fix_sub = n.subscribe("fix", 1, &RobotourControl::listenGpsFix, this);
}

void RobotourControl::listenGpsFix(const sensor_msgs::NavSatFixConstPtr &msg) {
    storedPosition.clear();
    storedPosition.push_back(msg->longitude);
    storedPosition.push_back(msg->latitude);
    gps_fix_sub.shutdown();

    Ui::ControlWidget::information_wiev->addItem("POSITION STORED");
    ROS_INFO("Longitude = %f", msg->longitude);
    ROS_INFO("Latitude   = %f", msg->latitude);
}

void RobotourControl::restorePosition_btn() {
    if(storedPosition.empty()) {
        Ui::ControlWidget::information_wiev->addItem("ANY POSITION STORED");
        return;
    }

    // change values in UI forms
    emit gpsValueChanged(storedPosition.at(0), storedPosition.at(1));
}

void RobotourControl::listenDiagnosticMsg(const diagnostic_msgs::DiagnosticArrayConstPtr &msg) {
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

void RobotourControl::updateGuiDiagnostic(QString battery1, QString battery2, QString current) {
    Ui::ControlWidget::battery1_label->setText(battery1);
    Ui::ControlWidget::battery2_label->setText(battery2);
    Ui::ControlWidget::current_label->setText(current);
}

void RobotourControl::ventilator_cbx() {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;

    bool_param.name = "wifi";
    bool_param.value = Ui::ControlWidget::ventilator_cbx->isChecked();

    conf.bools.push_back(bool_param);
    srv_req.config = conf;

    ros::service::call("/mrvk_driver/set_parameters", srv_req, srv_resp);
}

void RobotourControl::autoscroll(int state) {
    //autoscroll
    if(state){
        connect(Ui::ControlWidget::information_wiev->model(), SIGNAL(rowsInserted(QModelIndex,int,int)), Ui::ControlWidget::information_wiev, SLOT(scrollToBottom()));
    }
    else{
        disconnect(Ui::ControlWidget::information_wiev->model(), SIGNAL(rowsInserted(QModelIndex,int,int)), Ui::ControlWidget::information_wiev, SLOT(scrollToBottom()));
    }
}

void RobotourControl::setMap(){
    QString filename =  QFileDialog::getOpenFileName(
            nullptr,
            "Open Document",
            QDir::currentPath(),
            "Osm map (*.osm) ;; All files (*.*)");
    Ui::ControlWidget::map_label->setText(filename);

    //get map origin
    osm_planner::Parser parser;
    parser.setNewMap(filename.toStdString());
    std::vector<std::string> way_types;
    n.getParam("/move_base/Planner/filter_of_ways",way_types);
    //Set the density of points
    double interpolation_max_distance;
    n.param<double>("/move_base/Planner/interpolation_max_distance", interpolation_max_distance, 1000);
    parser.setInterpolationMaxDistance(interpolation_max_distance);

    parser.setTypeOfWays(way_types);
    parser.parse(false);
    map_origin = parser.getNodeByID(0);
    ROS_ERROR("kok");
    ROS_ERROR("OSM lat %f, lon %f", map_origin.latitude,map_origin.longitude);
}

void RobotourControl::init_btm(){
    // this service "init_robot" calibrate gyro and gps
    std_srvs::SetBool setbool_init;
    Ui::ControlWidget::information_wiev->addItem("Init");
    if (init_robot.call(setbool_init) && setbool_init.response.success) {
        Ui::ControlWidget::information_wiev->addItem("Init succesful");
    } else {
        ROS_ERROR_STREAM("Problem with robot inicialization and goal setup");
        Ui::ControlWidget::information_wiev->addItem("ERROR OCCURED WHILE INIT ROBOT");
    }
}