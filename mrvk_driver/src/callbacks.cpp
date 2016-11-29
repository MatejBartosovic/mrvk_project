/*
 * callbacks.cpp
 *
 *  Created on: 26.10.2016
 *      Author: michal
 */

#include <mrvk_driver/callbacks.h>
#include <errno.h>

MrvkCallbacks::MrvkCallbacks(CommunicationInterface *interface) {

	this->interface = interface;

	ros::NodeHandle n;

	//initSS = n.advertiseService("init", &MrvkCallbacks::initCallback, this);
	//resetFlagsSS = n.advertiseService("reset_flags", &MrvkCallbacks::resetFlagsCallback, this);
	shutdownSS = n.advertiseService("shutdown", &MrvkCallbacks::shutdownCallback, this);
	resetCentralStopSS = n.advertiseService("reset_central_stop", &MrvkCallbacks::resetCentralStopCallback, this);
	resetBatterySS = n.advertiseService("reset_Q_batery", &MrvkCallbacks::resetBatteryCallback, this);
	//stopSS = n.advertiseService("set_global_stop", &MrvkCallbacks::stopCallback, this);
	setArmVoltageSS = n.advertiseService("set_arm_voltage", &MrvkCallbacks::setArmVoltageCallback, this);
    toggleArmVoltageSS = n.advertiseService("toggle_arm_voltage", &MrvkCallbacks::toggleArmVoltageCallback, this);
    toggleCameraSourceSS = n.advertiseService("toggle_camera_source", &MrvkCallbacks::toggleCameraSourceCallback, this);
	setPowerManagmentSS = n.advertiseService("write_main_board_settings", &MrvkCallbacks::setPowerManagmentCallback, this);
	///setMotorParametersSS = n.advertiseService("write_motor_settings", &MrvkCallbacks::setMotorParametersCallback, this);

}

/*void MrvkCallbacks::resetCallbacksFlags() {

	inter = 0;
}*/

/*
bool MrvkCallbacks::resetFlagsCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {

		//resetFlags = true;
		//newCallback = true;

		res.success = true;
		res.message = "flags are reset";
		return true;
	}*/

	//todo overit funkcnost + ked bude na baterkach tak overit premennu full_battery
	bool MrvkCallbacks::resetBatteryCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        boost::unique_lock<boost::mutex> lock(interface->broadcast_mutex);
        interface->getMainBoard()->resetBatery();
        interface->broadcast.wait(lock);
        if (interface->succes & CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG) { res.success = true;
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

        //send motors request
        boost::unique_lock<boost::mutex> broadcast_lock(interface->broadcast_mutex);
        interface->getMotorControlBoardLeft()->setErrFlags(true,true);
        interface->getMotorControlBoardRight()->setErrFlags(true,true);
        interface->broadcast.wait(broadcast_lock);
        if((interface->succes & CommunicationInterface::MOTORS_BROADCAST_FLAG) != CommunicationInterface::MOTORS_BROADCAST_FLAG){
            res.message += "Motor boards write failed ";
            res.success = false;
            return true;
        }

        //send main board request
        interface->getMainBoard()->setCentralStop(false);
        interface->broadcast.wait(broadcast_lock);
        if((interface->succes & CommunicationInterface::ALL_BROADCAST_FLAG) != CommunicationInterface::ALL_BROADCAST_FLAG){
            res.message += "Main board write failed. ";
            res.success = false;
        }
        interface->broadcast_mutex.unlock();

        //wait to unblock
        boost::unique_lock<boost::mutex> lock(interface->data_mutex);
        for(int i =0;i<300;i++){
            interface->data.wait(lock);
            if(!interface->getStatus(&mrvk_driver::Mb_status::central_stop,true)) {
                res.success = true;
                res.message = "Ok";
                return true;
            }
        }
        res.message = "TIMEDOUT";
        res.success = false;
	}


	/*bool MrvkCallbacks::stopCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
	{
		//stop = true;
		////newCallback = true;
		 //stop();
		 res.success = true;
		 return true;
	}*/

	//100% funkcny servis
	bool MrvkCallbacks::setArmVoltageCallback(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res){

        //send request
        boost::unique_lock<boost::mutex> lock(interface->broadcast_mutex);
        interface->getMainBoard()->setArmPower(req.data);
        interface->broadcast.wait(lock);
        if((interface->succes & CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG) != CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG){
            interface->getMainBoard()->setArmPower(!req.data); //zmena spet
            res.message += "Main bosrd write failed. ";
            res.success = false;
            return true;
        }
        interface->broadcast_mutex.unlock();
        //check
        boost::unique_lock<boost::mutex> data_lock(interface->broadcast_mutex);
        for(int i = 0;i<12;i++){
            interface->data.wait(data_lock);
            if(interface->getPowerArm()==req.data){
                res.message = "Ok";
                res.success = true;
                return true;
            }
        }
        res.message = "TIMEDOUT";
        res.success = true;
	}

    bool MrvkCallbacks::toggleArmVoltageCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

        //send request
        boost::unique_lock<boost::mutex> lock(interface->broadcast_mutex);
        bool new_state = !interface->getPowerArm();
        interface->getMainBoard()->setArmPower(new_state);
        interface->broadcast.wait(lock);
        if((interface->succes & CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG) != CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG){
            interface->getMainBoard()->setArmPower(!new_state); //zmena spet
            res.message += "Main bosrd write failed. ";
            res.success = false;
            return true;
        }
        interface->broadcast_mutex.unlock();

        //check
        boost::unique_lock<boost::mutex> data_lock(interface->broadcast_mutex);
        for(int i = 0;i<12;i++){
            interface->data.wait(data_lock);
            if(interface->getPowerArm() == new_state){
                res.message = "Ok";
                res.success = true;
                return true;
            }
        }
            res.message = "TIMEDOUT";
            res.success = false;
    }

	//TODO overit funkcnost prerobit na toggle + set a kontroly
	bool MrvkCallbacks::toggleCameraSourceCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {

        //send request
        boost::unique_lock<boost::mutex> lock(interface->broadcast_mutex);
        bool ret = interface->getMainBoard()->switchVideo();
        interface->broadcast.wait(lock);
        if ((interface->succes & CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG) != CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG) {
            interface->getMainBoard()->switchVideo(); //zmena spet
            res.message = "Main boadr write failed. ";
            res.success = false;
            return true;
        }
        return true;
	}

	//100% funkcny servis
	bool MrvkCallbacks::setPowerManagmentCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
        if(interface->isActive()){
	    	SET_MAIN_BOARD config;
    		getMbFromParam(&config);

	    	boost::unique_lock<boost::mutex> lock(interface->broadcast_mutex);
    		interface->getMainBoard()->setParamatersMB(&config);
		    interface->broadcast.wait(lock);
            if((interface->succes & CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG) != CommunicationInterface::MAIN_BOARD_BROADCAST_FLAG){
                res.message = "Maind board write failed. If comunication is running correctly the settings will be written next cycle";
                res.success = false;
                return true;
            }
            res.success = true;
		    return true;
        }

        res.message = "Comunication is not active";
        res.success = false;
        return true;
	}

	/*bool MrvkCallbacks::setMotorParametersCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		//setMotorParameters = true;
		//newCallback = true;

		res.success = true;
		return true;
	}*/

/*	bool MrvkCallbacks::initCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		if (init())
			res.success = true;
		else res.success = false;

		return true;
	}*/

	bool MrvkCallbacks::shutdownCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		interface->close();
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
