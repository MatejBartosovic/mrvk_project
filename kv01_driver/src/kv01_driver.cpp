#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <kv01_driver/communication_interface.h>
#include "kv01_driver/hwInterface.h"

namespace Kv01{
    class Driver : public Kv01::HwInterface{

	public:
		Driver(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize):
                Kv01::HwInterface(),
				comunication_interface(ports, baudrate, stopBits, parity, byteSize)
		{
            last_time = ros::Time::now();
            current_time = ros::Time::now();
        }

		bool init(){

			ROS_INFO("Robot init");
			if (!comunication_interface.init())
				return false;
		/*todo spravit nejake inicializacie a zaciatocne nastavenia zatial nie je potrebne*/
			return true;
		}

		void read(){
            current_time = ros::Time::now();

			vel[0] = 0;
			vel[1] = 0;
			vel[2] = 0;
			vel[3] = 0;
			vel[4] = 0;
			vel[5] = 0;

			pos[0] = 0.2;
			pos[1] = 0.2;
			pos[2] = 0.2;
			pos[3] = 0.2;
			pos[4] = 0.2;
			pos[5] = 0.2;
		}

		void write(){
            last_time = current_time;
//			enforceLimits();
//			//ROS_ERROR("vel %lf %lf",vel[0],vel[1]);
//			//ROS_ERROR("vel cmd %lf %lf",vel_cmd[0],vel_cmd[1]);
//			comunication_interface.setMotorsVel(vel_cmd[0],vel_cmd[1]);
//			comunication_interface.setCameraPosition(pos_cmd[0],pos_cmd[1]);
//			comunication_interface.write();
//			comunication_interface.waitToRead();
		}

        ros::Duration getPeriod(){
            return current_time - last_time;
        }

	private:

		//MrvkCallbacks callbacks;
		CommunicationInterface comunication_interface;

		//diagnostic updater variables
		ros::Timer status_timer;
		//diagnostic_updater::Updater diagnostic;

        ros::Time last_time;
        ros::Time current_time;
	};
}

int main (int argc, char **argv){

	ros::init(argc, argv, "kv01_driver");
	ros::NodeHandle n;

	std::vector<std::string> ports;
	int baud;
	int stopBits;
	int byteSize;
	int parity;
	int rate;

	n.getParam("port_names",ports);
	n.param<int>("baudrate", baud, 115200);
	n.param<int>("stop_bits", stopBits, 1);
	n.param<int>("parity", parity, 1);
	n.param<int>("byte_size", byteSize, 8);
	n.param<int>("comunication_rate", rate, 50);

	Kv01::Driver driver(ports,baud,stopBits,parity,byteSize);
	driver.init();

	controller_manager::ControllerManager cm(&driver);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Rate a(rate);

	while (n.ok())
	{
		a.sleep();
		driver.read();
		cm.update(ros::Time::now(), driver.getPeriod());
		driver.write();
	}
	
return 0;
}
