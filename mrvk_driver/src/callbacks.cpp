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

    //dynamic reconfigure
    f = boost::bind(&MrvkCallbacks::dynamicReconfigureCallback, this, _1, _2);
    server.setCallback(f);
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
        return true;
	}

	bool MrvkCallbacks::resetCentralStopCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        uint8_t MCB_command[21];
        uint8_t MB_command[21];
        uint8_t request[5];
        int response;


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
            if(!communicationInterface.getWriteStatus(CommunicationInterface::AllDevicesFlag,write_lock)){
                res.message += "Main board write failed. ";
                res.success = false;
            }
        }

        //wait to unblock
        for(int i =0;i<100;i++){
            if(communicationInterface.getNewDataStatus(CommunicationInterface::MainBoardFlag)){
                if(!communicationInterface.getStatus(&mrvk_driver::Mb_status::central_stop)) {
                    communicationInterface.blockMovement(false);
                    res.success = true;
                    res.message = "Ok";
                    return true;
                }
            }
        }
        res.message = "TIMEOUT";
        res.success = false;
	}


	bool MrvkCallbacks::blockMovementCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
	{
		boost::unique_lock<boost::mutex> lock(communicationInterface.write_mutex);
        communicationInterface.blockMovement(req.data);
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
        res.message = "TIMEDOUT";
        res.success = true;
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
            res.message = "TIMEDOUT";
            res.success = false;
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

        res.message = "Comunication is not active";
        res.success = false;
        return true;
	}


	bool MrvkCallbacks::shutdownCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        communicationInterface.close();
		ros::shutdown();
		return true;
	}

	void MrvkCallbacks::getMbFromParam(SET_MAIN_BOARD *config){

		ros::NodeHandle n;
		n.param<bool>("MCBsSB_5V", config->MCBsSB_5V, true);
		n.param<bool>("MCBs_12V", config->MCBs_12V, true);
		n.param<bool>("wifi", config->wifi, true);
		n.param<bool>("video_transmitter", config->videoTransmitter, false);
		n.param<bool>("laser_scanner", config->laser, true);
		n.param<bool>("gps", config->GPS, false);
		n.param<bool>("pc2", config->PC2, true);
		n.param<bool>("camera", config->kamera, false);
		n.param<bool>("arm5V", config->ARM_5V, false);
		n.param<bool>("arm12V", config->ARM_12V, false);
		bool video;
		n.param<bool>("video", video, true);
		config->video1 = video;
		config->video2 = !video;

		int pko, pkk, iko, ikk;

		n.param<int>("kamera_PID_pko", pko, 10);
		n.param<int>("kamera_PID_pkk", pkk, 10);
		n.param<int>("kamera_PID_iko", iko, 40);
		n.param<int>("kamera_PID_ikk", ikk, 80);

		config->pko = pko;
		config->pkk = pkk;
		config->iko = iko;
		config->ikk = ikk;
	}

	void MrvkCallbacks::getMotorParametersFromParam(REGULATOR_MOTOR *reg, bool *regulation_type){

		ros::NodeHandle n;

		int ph, pl,ih, il;
		n.param<int>("motor_PID_ph",ph, 0);
		n.param<int>("motor_PID_pl",pl, 10);
		n.param<int>("motor_PID_ih",ih, 0);
		n.param<int>("motor_PID_il",il, 15);

		reg->PH = ph;
		reg->PL = pl;
		reg->IH = ih;
		reg->IL = il;

		//bool regulation_type;
		n.param<bool>("typ_regulacie_motora", *regulation_type, false);

	}

void MrvkCallbacks::dynamicReconfigureCallback(mrvk_driver::RobotDynParamConfig &config, uint32_t level) {

    ROS_ERROR("Reconfigure Request: %d %f %s %s %d",
             config.int_param, config.double_param,
             config.str_param.c_str(),
             config.arm_5v?"True":"False",
             config.size);
}
