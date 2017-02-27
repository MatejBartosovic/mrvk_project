#include <ros/ros.h>
#include <mrvk_driver/communication_interface.h>
#include <controller_manager/controller_manager.h>
#include <mrvk_driver/callbacks.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mrvk_driver/hwInterface.h>

namespace Mrvk{
	class Driver : public Mrvk::HwInterface{

	public:
		Driver(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize) :
					Mrvk::HwInterface(),
					comunication_interface(ports, baudrate, stopBits, parity, byteSize),
					callbacks(comunication_interface),
					diagnostic() {

			//setup diagnostic
			diagnostic.add("mrvk_driver Status", this, &Mrvk::Driver::diagnostics);
			diagnostic.setHardwareID("none");
			ros::NodeHandle n;
			double statusPeriod;
			n.param<double>("status_period", statusPeriod, 5);
			status_timer = n.createTimer(ros::Duration(statusPeriod), &Mrvk::Driver::statusTimerCallback, this);
            last_time = ros::Time::now();
            current_time = ros::Time::now();
        }

		bool init(){

			ROS_INFO("Robot init");

			if (!comunication_interface.init())
				return false;

			SET_MAIN_BOARD config;

			callbacks.getMbFromParam(&config);							//nastavi default parametre
			comunication_interface.getMainBoard()->setParamaters(&config);

			REGULATOR_MOTOR leftRegulator;
            REGULATOR_MOTOR rightRegulator;

			callbacks.getMotorParametersFromParam(&leftRegulator, &rightRegulator);			//nastavi default parametre
			comunication_interface.getMotorControlBoardLeft()->setRegulatorPID(leftRegulator);
            comunication_interface.getMotorControlBoardRight()->setRegulatorPID(rightRegulator);

			//todo dorobit odblokovanie central stopu do initu

			return true;
		}

		void read(){
            current_time = ros::Time::now();
			vel[0] = comunication_interface.getVelLeftWheel();
			vel[1] = -comunication_interface.getVelRightWheel();
            /*vel[0] = vel_cmd[0];
            vel[1] = vel_cmd[1];
            pos[0] += vel[0] * getPeriod().toSec();
            pos[1] += vel[1] * getPeriod().toSec();*/
			double lavy = -comunication_interface.getPosLeftWheel();
			double pravy = comunication_interface.getPosRightWheel();
			pos[0] += lavy;
			pos[1] += pravy;
			pos[3] = comunication_interface.getCameraPositionX();
			pos[4] = comunication_interface.getCameraPositionZ();
		}

		void write(){
            last_time = current_time;
			enforceLimits();
			//ROS_ERROR("vel %lf %lf",vel[0],vel[1]);
			//ROS_ERROR("vel cmd %lf %lf",vel_cmd[0],vel_cmd[1]);
			comunication_interface.setMotorsVel(vel_cmd[0],vel_cmd[1]);
			comunication_interface.setCameraPosition(pos_cmd[0],pos_cmd[1]);
			comunication_interface.write();
			comunication_interface.waitToRead();
		}

        ros::Duration getPeriod(){
            return current_time - last_time;
        }

	private:

		MrvkCallbacks callbacks;
		CommunicationInterface comunication_interface;

		//diagnostic updater variables
		ros::Timer status_timer;
		diagnostic_updater::Updater diagnostic;

        ros::Time last_time;
        ros::Time current_time;


		void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){

			if (comunication_interface.isActive()){

				stat.summary(0, "Connected");
				stat.add<mrvk_driver::Mb_status>("main board status",comunication_interface.getStatusMB());
				stat.add<mrvk_driver::Mcb_status>("left motor board status",comunication_interface.getStatusMCB(CommunicationInterface::LEFT_MOTOR_ADRESS));
				stat.add<mrvk_driver::Mcb_status>("right board status", comunication_interface.getStatusMCB(CommunicationInterface::RIGHT_MOTOR_ADRESS));

			}else stat.summary(2, "Disconnected");
		}

		void statusTimerCallback(const ros::TimerEvent& timer_struct) {
			diagnostic.update();
		}
	};
}

int main (int argc, char **argv){

	ros::init(argc, argv, "mrvk_driver");
	ros::NodeHandle n;

	std::vector<std::string> ports;
	int baud;
	int stopBits;
	int byteSize;
	int parity;

	n.getParam("port_names",ports);
	n.param<int>("baudrate", baud, 230400);
	n.param<int>("stop_bits", stopBits, 1);
	n.param<int>("parity", parity, 1);
	n.param<int>("byte_size", byteSize, 8);

	Mrvk::Driver driver(ports,baud,stopBits,parity,byteSize);
	driver.init();

	controller_manager::ControllerManager cm(&driver);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate a(10);

	while (n.ok())
	{
		a.sleep();
		driver.read();
		cm.update(ros::Time::now(), driver.getPeriod());
		driver.write();
	}
	
return 0;
}
