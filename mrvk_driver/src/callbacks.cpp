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
	setCameraSourceSS = n.advertiseService("camera_source", &MrvkCallbacks::setCameraSourceCallback, this);
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

	//todo overit funkcnost
	bool MrvkCallbacks::resetBatteryCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		boost::unique_lock<boost::mutex> lock(interface->callback_mutex);
		interface->getMainBoard()->resetBatery();
		interface->callback_condition.wait(lock);
		res.success = interface->succes;
		interface->callback_mutex.unlock();

		if (res.success)
			res.message = "battery is reset";
		else res.message = "battery is not reset";
		res.success = true;
		return true;
	}

	bool MrvkCallbacks::resetCentralStopCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		uint8_t MCB_command[21];
		uint8_t MB_command[21];
		uint8_t request[5];

		boost::unique_lock<boost::mutex> lock(interface->callback_mutex);
		interface->getMotorControlBoardLeft()->setErrFlags(true,true);
		interface->getMotorControlBoardRight()->setErrFlags(true,true);
		interface->callback_condition.wait(lock);
		res.success = interface->succes;
		interface->callback_mutex.unlock();

		interface->callback_mutex.lock();
		interface->getMainBoard()->setCentralStop(false);
		interface->callback_condition.wait(lock);
		res.success &= interface->succes;
		interface->callback_mutex.unlock();

		return true;
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
	bool MrvkCallbacks::setArmVoltageCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		bool success = false;
		static bool state = false;
		ros::NodeHandle n;

		boost::unique_lock<boost::mutex> lock(interface->callback_mutex);
		if (state){
			interface->getMainBoard()->setArmPower(false);
			interface->callback_condition.wait(lock);
			res.success = interface->succes;
			interface->callback_mutex.unlock();

			if (res.success){
				res.message = "Napajanie ramena vypnute";
				state = false;
				n.setParam("arm5V", false);
				n.setParam("arm12V", false);
			}
			else res.message = "error";
		}
		else{
			interface->getMainBoard()->setArmPower(true);
			interface->callback_condition.wait(lock);
			res.success = interface->succes;
			interface->callback_mutex.unlock();

			if (res.success){
				res.message = "Napajanie ramena zapnute";
				state = true;
				n.setParam("arm5V", true);
				n.setParam("arm12V", true);
			}
			else res.message = "error";
		}
		return true;
	}
	//todo overit funkcnost
	bool MrvkCallbacks::setCameraSourceCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res) {

		timespec timeout;
		timeout.tv_sec +=1;

		boost::unique_lock<boost::mutex> lock(interface->callback_mutex);
		bool ret = interface->getMainBoard()->switchVideo();
		interface->callback_condition.wait(lock);
		res.success = interface->succes;
		interface->callback_mutex.unlock();

		if(!res.success){
			interface->getMainBoard()->switchVideo(); // zmena spet
			res.message = "Video switch FAILED!!!!!";
		}
		//TODO dorobit vypis
		if(ret)
			res.message = "neviem kedy je zapnuta a kedy vypnuta";
		else
			res.message = "neviem kedy je zapnuta a kedy vypnuta";
		return true;
	}

	//100% funkcny servis
	bool MrvkCallbacks::setPowerManagmentCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){

		SET_MAIN_BOARD config;
		getMbFromParam(&config);

		boost::unique_lock<boost::mutex> lock(interface->callback_mutex);
		interface->getMainBoard()->setParamatersMB(&config);
		interface->callback_condition.wait(lock);
		res.success = interface->succes;
		interface->callback_mutex.unlock();
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
