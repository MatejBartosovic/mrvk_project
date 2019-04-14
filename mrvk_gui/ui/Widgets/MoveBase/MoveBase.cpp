#include "MoveBase.h"
#include <ui_MoveBase.h>
#include <ui_MoveBaseControl.h>


namespace mrvk_gui {
    MoveBase::MoveBase(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::MoveBase),
            actionClient("move_base", true){
        ui->setupUi(this);

        // load osm offsets
        ros::NodeHandle n;
        if (!n.getParam(ORIGIN_LATITUDE_PARAM_PATH, latitudeMapOffset)) {
            std::ostringstream oss;
            oss<<"Param: " <<ORIGIN_LATITUDE_PARAM_PATH << " is not on param server";
            QMessageBox msgBox(QMessageBox::Critical, "Param is not loaded", oss.str().c_str());
            msgBox.exec();
        }
        if (!n.getParam(ORIGIN_LONGTITUDE_PARAM_PATH, longitudeMapOffset)) {
            std::ostringstream oss;
            oss<<"Param: " <<ORIGIN_LONGTITUDE_PARAM_PATH << " is not on param server";
            QMessageBox msgBox(QMessageBox::Critical, "Param is not loaded", oss.str().c_str());
            msgBox.exec();
        }

        std::lconv* lc = std::localeconv();
        QRegExpValidator* latitudeValidator = new QRegExpValidator(QRegExp("^(\\+|-)?(?:90(?:(?:\\.0{1,6})?)|(?:[0-9]|[1-8][0-9])(?:(?:\\" + QString( lc->decimal_point) + "[0-9]{1,6})?))$"));
        ui->moveBaseControl->ui->latitudeGoalValue->setValidator(latitudeValidator);

        QRegExpValidator* longitudeValidator = new QRegExpValidator(QRegExp("^(\\+|-)?(?:180(?:(?:\\.0{1,6})?)|(?:[0-9]|[1-9][0-9]|1[0-7][0-9])(?:(?:\\"+ QString( lc->decimal_point) + "[0-9]{1,6})?))$"));
        ui->moveBaseControl->ui->longitudeGoalValue->setValidator(longitudeValidator);

//        loadDefaultMapOffset();

        connect(ui->moveBaseControl->ui->goButton,SIGNAL(released()),this,SLOT(goSlot()));
        connect(ui->moveBaseControl->ui->cancelButton,SIGNAL(released()),this,SLOT(cancelSlot()));
        connect(ui->moveBaseControl->ui->mapOffsetButton,SIGNAL(released()),this,SLOT(editMapOffsetSlot()));
        connect(ui->moveBaseControl->ui->loadQrCodeButton,SIGNAL(released()),this,SLOT(readQrCodeSlot()));
        connect(ui->moveBaseControl->ui->goalByOffset_cbx, SIGNAL(clicked(bool)), this, SLOT(goByOffsetCbx(bool)));

    }

    MoveBase::~MoveBase() {
        delete ui;
    }

    void MoveBase::updateData(){
        ui->moveBaseStatus->updateData();
        ui->moveBaseControl->updateData();
    }

    void MoveBase::goSlot(){
        if(!actionClient.isServerConnected() &&  !actionClient.waitForServer(ros::Duration(0.5))){
            QMessageBox box(QMessageBox::Icon::Warning,"Action client","Action server is not running. Aborting goal.",QMessageBox::Button::Ok,this);
            box.exec();
            return;
        }

        if(ui->moveBaseControl->ui->latitudeGoalValue->text().isEmpty() || ui->moveBaseControl->ui->longitudeGoalValue->text().isEmpty()){
            return;
        }

        auto goalTimestamp = ros::Time::now();
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.stamp = goalTimestamp;
        goal.target_pose.header.frame_id = TF_WORLD_FRAME;

        if (setGoalByOffset) {
            // user set goal by local offset in robot space

            tf::TransformListener listener;
            tf::StampedTransform baseLinkWorldTF;

            try{
                listener.waitForTransform(TF_BASE_LINK_FRAME, TF_WORLD_FRAME, goalTimestamp, ros::Duration(1));
                listener.lookupTransform(TF_BASE_LINK_FRAME, TF_WORLD_FRAME, goalTimestamp, baseLinkWorldTF);
            }
            catch (tf::TransformException &ex){
                std::ostringstream oss;
                oss << "Goal has not been set. Can't find transform between child [" << TF_BASE_LINK_FRAME <<"] and robot base_link frame ["
                    << TF_WORLD_FRAME << "]. Exception:" << ex.what();
                ROS_ERROR(oss.str().c_str());
                QMessageBox box(QMessageBox::Critical, "Tf", oss.str().c_str());
                box.exec();
                return;
            }

            auto x = ui->moveBaseControl->ui->latitudeGoalValue->text().toDouble();
            auto y = ui->moveBaseControl->ui->longitudeGoalValue->text().toDouble();
            tf::Transform result = baseLinkWorldTF * tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(x, y, 0));
            tf::poseTFToMsg(result, goal.target_pose.pose);

        } else {
            // user set goal by geo space

            tf::quaternionEigenToMsg(Eigen::Quaterniond(1,0,0,0), goal.target_pose.pose.orientation);

            osm_planner::coordinates_converters::GeoNode geoNode = {std::stod(ui->moveBaseControl->ui->latitudeGoalValue->text().toStdString()) ,
                                                                    std::stod(ui->moveBaseControl->ui->longitudeGoalValue->text().toStdString()) ,0,0}; //TODO init

            std::cout << geoNode.latitude << "  "<<geoNode.longitude << std::endl;

            osm_planner::coordinates_converters::HaversineFormula converter;
            converter.setOrigin(latitudeMapOffset,longitudeMapOffset);
            goal.target_pose.pose.position.x = converter.getCoordinateX(geoNode);
            goal.target_pose.pose.position.y = converter.getCoordinateY(geoNode);
            goal.target_pose.pose.position.z = 0;
        }

        ui->moveBaseStatus->setGoal(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        actionClient.sendGoal(goal,boost::bind(&MoveBaseStatus::doneCallback, ui->moveBaseStatus, _1, _2),
                boost::bind(&MoveBaseStatus::activeCallback, ui->moveBaseStatus),
                boost::bind(&MoveBaseStatus::feedbackCallback, ui->moveBaseStatus, _1));
    }

    void MoveBase::cancelSlot(){
        actionClient.cancelGoal();
    }

    void MoveBase::editMapOffsetSlot(){

        GpsCoordinatesInput input(latitudeMapOffset, longitudeMapOffset);
        input.exec();
        if(input.result() == GpsCoordinatesInput::Rejected){
            return;
        }
        latitudeMapOffset = input.getLatitude();
        longitudeMapOffset = input.getLongitude();
//        saveDefaultMapOffset();
    }

    void MoveBase::readQrCodeSlot(){
        QMessageBox box(QMessageBox::Icon::Information,"Feature not implemented","Feature not implemented",QMessageBox::Button::Ok,this);
        box.exec();
    }

//    void MoveBase::loadDefaultMapOffset(){
//        const char* home = std::getenv("HOME");
//        if(home) {
//            std::ifstream file;
//            file.open(std::string(home) + "/.mrvk_gui.conf");
//            file >> latitudeMapOffset;
//            file >> longitudeMapOffset;
//            file.close();
//        }
//    }
//    void MoveBase::saveDefaultMapOffset(){
//        const char* home = std::getenv("HOME");
//        if(home) {
//            std::ofstream file;
//            file.open(std::string(home) + "/.mrvk_gui.conf");
//            file << std::fixed << std::setprecision(6) << latitudeMapOffset << std::endl <<longitudeMapOffset<<std::endl;
//            file.close();
//        }
//    }
}

void mrvk_gui::MoveBase::goByOffsetCbx(bool value) {
    setGoalByOffset = value;

    if (value) {
        ui->moveBaseControl->ui->latitudeTitle->setText("X");
        ui->moveBaseControl->ui->longitudTitle->setText("Y");

        // disable some items
        ui->moveBaseControl->ui->coppyrStartToGoalButton->setDisabled(true);
    } else {
        ui->moveBaseControl->ui->latitudeTitle->setText("Latitude");
        ui->moveBaseControl->ui->longitudTitle->setText("Longitude");

        // enable some items
        ui->moveBaseControl->ui->coppyrStartToGoalButton->setDisabled(false);
    }
}