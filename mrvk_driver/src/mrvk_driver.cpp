#include <ros/ros.h>
#include <mrvk_driver/communication_interface.h>

#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <signal.h>
#include <mrvk_driver/HwInterface.h>
#include <controller_manager/controller_manager.h>
#include <mrvk_driver/callbacks.h>

#include <diagnostic_updater/diagnostic_updater.h>

class MrvkDriver{

public:
	MrvkDriver() : mrvkHW(),diagnostic_( ){

		if(mrvkHW.init(&hw, &robot_transmissions) !=4 ){
			ROS_ERROR("init nezbehol");
			ros::shutdown();
		}

		joint_to_vel_act = robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>();
		joint_to_pos_act = robot_transmissions.get<transmission_interface::JointToActuatorPositionInterface>();
		act_to_joint_state = robot_transmissions.get<transmission_interface::ActuatorToJointStateInterface>();

		this->velocity = mrvkHW.getVelVector();
		this->velocity_command = mrvkHW.getVelCmdVector();
        this->position = mrvkHW.getPosVector();
        this->position_command = mrvkHW.getPosCmdVector();

		limits = mrvkHW.getLimits();
		for(int i=0; i< 2; i++){
			//ROS_ERROR("%s",limits[i].joint.c_str());
			jnt_limit_interface.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(hw.get<hardware_interface::VelocityJointInterface>()->getHandle(limits[i].joint),limits[i].joint_info));
		}
        for(int i= 2; i<4;i++){
            camera_limit_interface.registerHandle(joint_limits_interface::PositionJointSaturationHandle(hw.get<hardware_interface::PositionJointInterface>()->getHandle(limits[i].joint),limits[i].joint_info));
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

		diagnostic_.add("mrvk_driver Status", this, &MrvkDriver::diagnostics);
	    diagnostic_.setHardwareID("none");

	    double statusPeriod;
	    n.param<double>("status_period", statusPeriod, 5);
		status_timer = n.createTimer(ros::Duration(statusPeriod), &MrvkDriver::statusTimerCallback, this);
	}

	bool init(){

		ROS_INFO("Robot init");

		if (!comunication_interface->init())
			return false;

		SET_MAIN_BOARD config;

		callbacks->getMbFromParam(&config);							//nastavi default parametre
		comunication_interface->getMainBoard()->setParamaters(&config);

		REGULATOR_MOTOR regulator;
		bool regulation_type;
		callbacks->getMotorParametersFromParam(&regulator, &regulation_type);			//nastavi default parametre
		comunication_interface->setMotorParameters(regulator, regulation_type);

		//todo dorobit odblokovanie central stopu do initu
		last_time = ros::Time::now();
		current_time = ros::Time::now();

		 return true;
	}

	void read(){
		velocity->at(0) = comunication_interface->getSpeedLeftWheel();
		velocity->at(1) = -comunication_interface->getSpeedRightWheel();
		position->at(0) = position->at(0) + velocity->at(0) * getPeriod();
		position->at(1) = position->at(1) + velocity->at(1) * getPeriod();
		act_to_joint_state->propagate();
		statusMutex.lock();
		mainBoardStatus = comunication_interface->getStatusMB();
		leftMotorStatus = comunication_interface->getStatusMCB(CommunicationInterface::LEFT_MOTOR_ADRESS);
		rightMotorStatus = comunication_interface->getStatusMCB(CommunicationInterface::RIGHT_MOTOR_ADRESS);
		statusMutex.unlock();
	}

	void write(){
		propagataAndEnforceLimits();  // spocitaj rychlosti z motora cez prevodovku a dodrz limity
		comunication_interface->setMotorsVel(velocity_command->at(0),velocity_command->at(1));
		comunication_interface->setCameraPosition(position_command->at(2),position_command->at(3));
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

	inline void propagataAndEnforceLimits(){
		current_time = ros::Time::now();
		jnt_limit_interface.enforceLimits(current_time - last_time);
		camera_limit_interface.enforceLimits(current_time - last_time);
		last_time = current_time;
		joint_to_vel_act->propagate();
		joint_to_pos_act->propagate();
	}

	int publish_rate;
	std::vector<double>* velocity;
	std::vector<double>* velocity_command;
    std::vector<double>* position;
    std::vector<double>* position_command;
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

	transmission_interface::JointToActuatorVelocityInterface* joint_to_vel_act;
	transmission_interface::JointToActuatorPositionInterface* joint_to_pos_act;
	transmission_interface::ActuatorToJointStateInterface* act_to_joint_state;


	joint_limits_interface::VelocityJointSaturationInterface jnt_limit_interface;
    joint_limits_interface::PositionJointSaturationInterface camera_limit_interface;

	std::vector<mrvk::Limits> limits;

	mrvk_driver::Mb_status mainBoardStatus;
	mrvk_driver::Mcb_status leftMotorStatus;
	mrvk_driver::Mcb_status rightMotorStatus;

	boost::mutex statusMutex;
	ros::Timer status_timer;
	diagnostic_updater::Updater diagnostic_;

	void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){


		if (comunication_interface->isActive()){

			stat.summary(0, "Connected");

			 statusMutex.lock();
			 stat.add<mrvk_driver::Mb_status>("main board status",mainBoardStatus);
			 stat.add<mrvk_driver::Mcb_status>("left motor board status",leftMotorStatus);
			 stat.add<mrvk_driver::Mcb_status>("right board status",rightMotorStatus);
			 statusMutex.unlock();

		}else stat.summary(2, "Disconnected");

	}

	void statusTimerCallback(const ros::TimerEvent& timer_struct) {

		diagnostic_.update();
	}

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
	}
	
return 0;
}
