#include <ros/ros.h>
#include <mrvk_driver/communication_interface.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <signal.h>
#include <mrvk_driver/HwInterface.h>
#include <controller_manager/controller_manager.h>
#include <mrvk_driver/callbacks.h>


//todo spravit nastavenie parametrov motora
//polohovanie kamery
//rozdelit komunikaciu na 3 porty

class MrvkDriver{

public:
	MrvkDriver() : mrvkHW(){

		if(mrvkHW.init(&hw, &robot_transmissions) != 2){
			ROS_ERROR("init nezbehol");
			ros::shutdown();
		}

		joint_to_act = robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>();
		act_to_joint = robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>();
		this->velocity = mrvkHW.getVelVector();
		this->velocity_command = mrvkHW.getVelCmdVector();

		limits = mrvkHW.getLimits();
		for(int i=0; i< limits.size(); i++){
			ROS_ERROR("%s",limits[i].joint.c_str());
			jnt_limit_interface.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(hw.get<hardware_interface::VelocityJointInterface>()->getHandle(limits[i].joint),limits[i].joint_info));
		}
		BOOST_FOREACH(std::string name,jnt_limit_interface.getNames()){
			ROS_ERROR("mam %s",name.c_str());		
		}
		std::vector<std::string> ports;
		int baud;
		int stopBits;
		int byteSize;
		int parity;

		ros::NodeHandle n;

		n.getParam("port_names",ports);
		n.param<int>("baudrate", baud, 230400);
		n.param<int>("stop_bits", stopBits, 1);
		n.param<int>("parity", parity, 1);
		n.param<int>("byte_size", byteSize, 8);
		n.param<int>("publish_rate", publish_rate, 10);

		comunication_interface = new CommunicationInterface(ports, baud, stopBits, parity, byteSize);
		callbacks = new MrvkCallbacks(comunication_interface);

		//TODO toto prec Miso: zatial to tu chcem nechat potom by som spravil diagnostic updater
		pub_MCBlStatus = n.advertise<mrvk_driver::Mcb_status>("motor_status_left", 10); //publishery s konkretnou statusovou spravou
		pub_MCBrStatus = n.advertise<mrvk_driver::Mcb_status>("motor_status_right", 10); //publishery s konkretnou statusovou spravou
		pub_MBStatus = n.advertise<mrvk_driver::Mb_status>("main_board_status", 10);

	}

	bool init(){

		ROS_ERROR("robot init"); //TODO prerobit na info alebo prec

		if (!comunication_interface->init())
			return false;

		SET_MAIN_BOARD config;

		//TODO funkciu setMbFromParam presunut do comunication interface a prerobit set mainboart aby si to pitala sama
		callbacks->getMbFromParam(&config);							//nastavi default parametre
		comunication_interface->setMainBoard(&config);

		/*REGULATOR_MOTOR regulator;
		bool regulation_type;
		callbacks->setMotorParametersFromParam(&regulator, &regulation_type);			//nastavi default parametre
		comunication_interface->setMotorParameters(regulator, regulation_type);*/

		//TODO centralstop spojazdnit
		/*if (!comunication_interface->resetCentralStop())
			return false;*/
		REGULATOR_MOTOR regulator;
		bool regulation_type;
		callbacks->getMotorParametersFromParam(&regulator, &regulation_type);			//nastavi default parametre
		comunication_interface->setMotorParameters(regulator, regulation_type);
		last_time = ros::Time::now();
		current_time = ros::Time::now();

		 return true;

	}

	void read(){
		velocity->at(0) = comunication_interface->getSpeedLeftWheel();
		velocity->at(1) = -comunication_interface->getSpeedRightWheel();
		//velocity->at(0) = velocity_command->at(0);
		//velocity->at(1) = velocity_command->at(1);
		act_to_joint->propagate();	
		pub_MCBlStatus.publish(comunication_interface->getStatusMCB(CommunicationInterface::LEFT_MOTOR_ADRESS));
		pub_MCBrStatus.publish(comunication_interface->getStatusMCB(CommunicationInterface::RIGHT_MOTOR_ADRESS));
		pub_MBStatus.publish(comunication_interface->getStatusMB());
	}

	void write(){
		current_time = ros::Time::now();
		jnt_limit_interface.enforceLimits(current_time - last_time); //TODO ros::duration prerobit
		last_time = current_time;
		joint_to_act->propagate();
		//ROS_ERROR("pravy vel =%lf cmd = %lf",velocity->at(1),velocity_command->at(1));
		comunication_interface->setMotorsVel(velocity_command->at(0),velocity_command->at(1));
		/*comunication_interface->writeMB();
		comunication_interface->writeMotors();*/
		comunication_interface->write();
		comunication_interface->waitToRead();
	}

	int getFrequence(){

		return publish_rate;
	}

	double getPeriod(){

		return 1.0/publish_rate;
	}
	hardware_interface::RobotHW* getHw(){
		return &hw;
	}

private:

	int publish_rate;
	std::vector<double>* velocity;
	std::vector<double>* velocity_command;
	MrvkCallbacks *callbacks;

	ros::Time last_time;
	ros::Time current_time;

	CommunicationInterface *comunication_interface;

	ros::Publisher pub_MCBlStatus;
	ros::Publisher pub_MCBrStatus;
	ros::Publisher pub_MBStatus;

	hardware_interface::RobotHW hw;
	mrvk::HwInterface mrvkHW;
	transmission_interface::RobotTransmissions robot_transmissions;

	transmission_interface::JointToActuatorVelocityInterface* joint_to_act;
	transmission_interface::ActuatorToJointStateInterface* act_to_joint;
	joint_limits_interface::VelocityJointSaturationInterface jnt_limit_interface;
	std::vector<mrvk::Limits> limits;

};


int main (int argc, char **argv){

	ros::init(argc, argv, "mrvk_driver");
	ros::NodeHandle n;

	MrvkDriver driver;
	driver.init();

	controller_manager::ControllerManager cm(driver.getHw());

	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::Rate rate(driver.getFrequence());
	ROS_ERROR("PERIOD = %lf",driver.getPeriod());
	
	while (n.ok())
	{
		rate.sleep();
		driver.read();
		cm.update(ros::Time::now(), ros::Duration(driver.getPeriod()));
		driver.write();
		rate.sleep();
	}
	
return 0;
}
