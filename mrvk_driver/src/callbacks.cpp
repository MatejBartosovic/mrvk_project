/*
 * callbacks.cpp
 *
 *  Created on: 26.10.2016
 *      Author: michal
 */

#include <mrvk_driver/callbacks.h>

MrvkCallbacks::MrvkCallbacks(CommunicationInterface &interface) : communicationInterface(interface) {

	ros::NodeHandle n;

	shutdownSS = n.advertiseService("shutdown", &MrvkCallbacks::shutdownCallback, this);
	resetCentralStopSS = n.advertiseService("reset_central_stop", &MrvkCallbacks::resetCentralStopCallback, this);
	resetBatterySS = n.advertiseService("reset_Q_batery", &MrvkCallbacks::resetBatteryCallback, this);
	blockMovementSS = n.advertiseService("block_movement", &MrvkCallbacks::blockMovementCallback, this);
	setArmVoltageSS = n.advertiseService("set_arm_voltage", &MrvkCallbacks::setArmVoltageCallback, this);
    toggleArmVoltageSS = n.advertiseService("toggle_arm_voltage", &MrvkCallbacks::toggleArmVoltageCallback, this);
    toggleCameraSourceSS = n.advertiseService("toggle_camera_source", &MrvkCallbacks::toggleCameraSourceCallback, this);
	setPowerManagmentSS = n.advertiseService("write_main_board_settings", &MrvkCallbacks::setPowerManagmentCallback, this);
    setCentralStopSS = n.advertiseService("setCentralStop", &MrvkCallbacks::setCentralStop,this);

    //dynamic reconfigure
	boost::thread a = boost::thread(&MrvkCallbacks::createDynamicReconf,this);
	a.detach();

}

	//todo overit funkcnost + ked bude na baterkach tak overit premennu full_battery
	bool MrvkCallbacks::resetBatteryCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
        communicationInterface.getMainBoard()->resetBatery();
        if (communicationInterface.getWriteStatus(CommunicationInterface::MainBoardFlag,lock)) {
            res.success = true;
            res.message = "Battery reseted";
            return true;
        } else {
            res.success = true;
            res.message = "Write Failed";
            return true;
        }
	}

	bool MrvkCallbacks::resetCentralStopCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
		{
            //send motors request
            boost::unique_lock<boost::mutex> write_lock(communicationInterface.write_mutex);
            communicationInterface.blockMovement(true);
            communicationInterface.getMotorControlBoardLeft()->setErrFlags(true,true);
            communicationInterface.getMotorControlBoardRight()->setErrFlags(true,true);
            if(!communicationInterface.getWriteStatus(CommunicationInterface::MotorBoardsFlag,write_lock)){
                res.message += "Motor boards write failed ";
                res.success = false;
                return true;
            }

            //send main board request
            communicationInterface.getMainBoard()->setCentralStop(false);
            if(!communicationInterface.getWriteStatus(CommunicationInterface::MainBoardFlag,write_lock)) {
                res.message += "Main board write failed. ";
                res.success = false;
            }
		}
        //wait to unblock
        for(int i =0;i<100;i++){
            if(communicationInterface.getNewDataStatus(CommunicationInterface::MainBoardFlag)){
                if(!communicationInterface.getStatusMB(&mrvk_driver::Mb_status::central_stop)) {
                    communicationInterface.blockMovement(false);
                    res.success = true;
                    res.message = "Ok";
                    return true;
                }
            }
        }
        if(communicationInterface.getStatusMB(&mrvk_driver::Mb_status::hardware_central_stop)){
            res.success = false;
            res.message = "hardware central stop active";
            return true;
        }
        res.message = "TIMEOUT";
        res.success = false;
        return true;
	}

    bool MrvkCallbacks::setCentralStop(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
        boost::unique_lock<boost::mutex> write_lock(communicationInterface.write_mutex);
        communicationInterface.blockMovement(true);
        communicationInterface.getMainBoard()->setCentralStop(true);
        if(!communicationInterface.getWriteStatus(CommunicationInterface::MotorBoardsFlag,write_lock)){
            res.message += "Main board write failed ";
            res.success = false;
            return true;
        }
        return true;
    }


	bool MrvkCallbacks::blockMovementCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
	{
		boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
        communicationInterface.blockMovement(req.data);
		res.success = true;
		return true;
	}


	//100% funkcny servis
	bool MrvkCallbacks::setArmVoltageCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){

        //send request
        {
            boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
            communicationInterface.getMainBoard()->setArmPower(req.data);
            if (!communicationInterface.getWriteStatus(CommunicationInterface::MainBoardFlag, lock)) {
                communicationInterface.getMainBoard()->setArmPower(!req.data); //zmena spet
                res.message += "Main bosrd write failed. ";
                res.success = false;
                return true;
            }
        }

        for(int i = 0;i<12;i++){
            if(communicationInterface.getNewDataStatus(CommunicationInterface::MainBoardFlag)){
                if(communicationInterface.getPowerArm()==req.data){
                    res.message = "Ok";
                    res.success = true;
                    return true;
                }
            }
        }
        res.message = "TIMEOUT";
        res.success = false;
        return true;
	}

    bool MrvkCallbacks::toggleArmVoltageCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        //send request
        bool new_state;
        {
            boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
            new_state = !communicationInterface.getPowerArm();
            communicationInterface.getMainBoard()->setArmPower(new_state);
            if (!communicationInterface.getWriteStatus(CommunicationInterface::MainBoardFlag, lock)) {
                communicationInterface.getMainBoard()->setArmPower(!new_state); //zmena spet
                res.message += "Main bosrd write failed. ";
                res.success = false;
                return true;
            }
        }

        //check
        for(int i = 0;i<12;i++){
            if(communicationInterface.getNewDataStatus(CommunicationInterface::MainBoardFlag)){
                if(communicationInterface.getPowerArm() == new_state){
                    res.message = "Ok";
                    res.success = true;
                    return true;
                }
            }
        }
            res.message = "TIMEOUT";
            res.success = false;
             return true;
    }

	//TODO overit funkcnost prerobit na toggle + set a kontroly
	bool MrvkCallbacks::toggleCameraSourceCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {

        //send request
        boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
        communicationInterface.getMainBoard()->switchVideo();
        if (!communicationInterface.getWriteStatus(CommunicationInterface::MainBoardFlag,lock)) {
            communicationInterface.getMainBoard()->switchVideo(); //zmena spet
            res.message = "Main boadr write failed. ";
            res.success = false;
            return true;
        }
        return true;
	}

	//100% funkcny servis
	bool MrvkCallbacks::setPowerManagmentCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
	    	SET_MAIN_BOARD config;
    		getMbFromParam(&config);

	    	boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
            communicationInterface.getMainBoard()->setParamaters(&config);
            if(!communicationInterface.getWriteStatus(CommunicationInterface::MainBoardFlag,lock)){
                res.message = "Maind board write failed.";
                res.success = false;
                return true;
            }
            res.success = true;
		    return true;
	}

	bool MrvkCallbacks::shutdownCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        communicationInterface.close();
		ros::shutdown();
		return true;
	}

	void MrvkCallbacks::getMbFromParam(SET_MAIN_BOARD *config){

		ros::NodeHandle n;
		n.param<bool>("mcb_5V", config->MCBsSB_5V, true);
		n.param<bool>("mcb_12V", config->MCBs_12V, true);
		n.param<bool>("fan", config->wifi, true);
		n.param<bool>("video_transmitter", config->videoTransmitter, false);
		n.param<bool>("laser_scanner", config->laser, true);
		n.param<bool>("gps", config->GPS, false);
		n.param<bool>("pc2", config->PC2, true);
		n.param<bool>("camera", config->kamera, false);
		n.param<bool>("arm_5V", config->ARM_5V, false);
		n.param<bool>("arm_12V", config->ARM_12V, false);
		bool video;
		n.param<bool>("video", video, true);
		config->video1 = video;
		config->video2 = !video;

		int pko, pkk, iko, ikk;

		n.param<int>("camera_pko", pko, 10);
		n.param<int>("camera_pkk", pkk, 10);
		n.param<int>("camera_iko", iko, 40);
		n.param<int>("camera_ikk", ikk, 80);

		config->pko = (uint8_t) pko;
		config->pkk = (uint8_t) pkk;
		config->iko = (uint8_t) iko;
		config->ikk = (uint8_t) ikk;
	}

	void MrvkCallbacks::getMotorParametersFromParam(REGULATOR_MOTOR *left_reg, REGULATOR_MOTOR *right_reg){

		ros::NodeHandle n;

		int left_P, left_I, right_P, right_I;
        bool control;

		n.param<int>("left_P",left_P, 10);
		n.param<int>("left_I",left_I, 15);
        n.param<int>("right_P",right_P, 10);
        n.param<int>("right_I",right_I, 15);

        n.param<bool>("pwm_control", control, false);
        left_reg->pwm_control = control;
        right_reg->pwm_control = control;


        left_reg->PL = ((uint16_t) left_P ) & 0x00FF;
        left_reg->PH = (((uint16_t) left_P) & 0xFF00) >> 8;
        left_reg->IL = ((uint16_t) left_I ) & 0x00FF;
        left_reg->IH = (((uint16_t) left_I) & 0xFF00) >> 8;

        right_reg->PL = ((uint16_t) right_P ) & 0x00FF;
        right_reg->PH = (((uint16_t) right_P) & 0xFF00) >> 8;
        right_reg->IL = ((uint16_t) right_I ) & 0x00FF;
        right_reg->IH = (((uint16_t) right_I) & 0xFF00) >> 8;

	}

void MrvkCallbacks::dynamicReconfigureCallback(mrvk_driver::RobotDynParamConfig &config, uint32_t level) {

	//ROS_ERROR("dynamic reconigure callback start ");
    SET_MAIN_BOARD robot_config;
    REGULATOR_MOTOR left_reg;
    REGULATOR_MOTOR right_reg;

    robot_config.MCBsSB_5V = config.mcb_5V;
    robot_config.MCBs_12V = config.mcb_12V;
    robot_config.wifi = config.fan;
    robot_config.GPS = config.gps;
    robot_config.laser = config.laser_scanner;
    robot_config.videoTransmitter = config.video_transmitter;
    robot_config.ARM_5V = config.arm_5V;
    robot_config.ARM_12V = config.arm_12V;
    robot_config.PC2 = config.pc2;

    robot_config.video1 = config.video;
    robot_config.video2 = !config.video;

    robot_config.pko = config.camera_pko;
    robot_config.pkk = config.camera_pkk;
    robot_config.iko = config.camera_iko;
    robot_config.ikk = config.camera_ikk;

    left_reg.PL = ((uint16_t) config.left_P ) & 0x00FF;
    left_reg.PH = (((uint16_t) config.left_P) & 0xFF00) >> 8;
    left_reg.IL = ((uint16_t) config.left_I ) & 0x00FF;
    left_reg.IH = (((uint16_t) config.left_I) & 0xFF00) >> 8;

    right_reg.PL = ((uint16_t) config.right_P ) & 0x00FF;
    right_reg.PH = (((uint16_t) config.right_P) & 0xFF00) >> 8;
    right_reg.IL = ((uint16_t) config.right_I ) & 0x00FF;
    right_reg.IH = (((uint16_t) config.right_I) & 0xFF00) >> 8;

    //iba otackova regulacia
    left_reg.pwm_control = false;
    right_reg.pwm_control = false;

    boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
    communicationInterface.getMotorControlBoardLeft()->setRegulatorPID(left_reg);
    communicationInterface.getMotorControlBoardLeft()->setRegulatorPID(right_reg);
    communicationInterface.getMainBoard()->setParamaters(&robot_config);

    if(!communicationInterface.getWriteStatus(CommunicationInterface::AllDevicesFlag,lock)){
      ROS_ERROR("Parameters write failed.");
    }
}

void MrvkCallbacks::createDynamicReconf(){
	server.setCallback(boost::bind(&MrvkCallbacks::dynamicReconfigureCallback, this, _1, _2));
}
