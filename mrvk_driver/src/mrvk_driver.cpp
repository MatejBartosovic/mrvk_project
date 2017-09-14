#include <ros/ros.h>
#include <mrvk_driver/communication_interface.h>
#include <controller_manager/controller_manager.h>
#include <mrvk_driver/callbacks.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <mrvk_driver/hwInterface.h>
#include <thread>

namespace Mrvk{
	class Driver : public Mrvk::HwInterface{

	public:
		Driver(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize) :
					Mrvk::HwInterface(),
					comunication_interface(ports, baudrate, stopBits, parity, byteSize),
					callbacks(comunication_interface),
					diagnostic(),
					lastStopButtonVal(false) {

			//setup diagnostic
			diagnostic.add("mrvk_driver Status", this, &Mrvk::Driver::diagnostics);
			diagnostic.setHardwareID("none");
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

			//inicializacia je este pred 1. zapisom do robota v callbacku dynamic reconfigure

			SET_MAIN_BOARD config;

			callbacks.getMbFromParam(&config);							//nastavi default parametre
			comunication_interface.getMainBoard()->setParamaters(&config);

			REGULATOR_MOTOR leftRegulator;
            REGULATOR_MOTOR rightRegulator;

			callbacks.getMotorParametersFromParam(&leftRegulator, &rightRegulator);			//nastavi default parametre
			comunication_interface.getMotorControlBoardLeft()->setRegulatorPID(leftRegulator);
            comunication_interface.getMotorControlBoardRight()->setRegulatorPID(rightRegulator);

			//reset centralstop on startup
			thread = std::thread([=](){
				callResetCentralStopService();
			});
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
			//ROS_ERROR("l = %lf  r = %lf",lavy,pravy);
			//static int start_read_blog = 0;
			//if(start_read_blog > 10){
			pos[0] += lavy;
			pos[1] += pravy;
			//}
			//else{
			//start_read_blog++;
			//pos[0] = 0;
			//pos[1] = 0;
			//}
			
			pos[3] = comunication_interface.getCameraPositionX();
			pos[4] = comunication_interface.getCameraPositionZ();
		}

		void write(){
            last_time = current_time;
			enforceLimits();
			//ROS_ERROR("vel %lf %lf",vel[0],vel[1]);
			//ROS_ERROR("vel cmd %lf %lf",vel_cmd[0],vel_cmd[1]);
			comunication_interface.setMotorsVel(vel_cmd[0],vel_cmd[1]);
			//ROS_ERROR("% lf %lf",pos_cmd[0],pos_cmd[1]);
			comunication_interface.setCameraPosition(pos_cmd[0],pos_cmd[1]);
			comunication_interface.write();
			comunication_interface.waitToRead();
		}

        ros::Duration getPeriod(){
            return current_time - last_time;
        }

	private:

		CommunicationInterface comunication_interface;
		MrvkCallbacks callbacks;
		std::thread thread;


		//diagnostic updater variables
		ros::Timer status_timer;
		diagnostic_updater::Updater diagnostic;

        ros::Time last_time;
        ros::Time current_time;
		ros::NodeHandle n;
		bool lastStopButtonVal;

		void diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){

			if (comunication_interface.isActive()){

				stat.summary(0, "Connected");
				stat.add<mrvk_driver::Mb_status>("main board status",comunication_interface.getStatusMB());
				stat.add<mrvk_driver::Mcb_status>("left motor board status",comunication_interface.getStatusMCB(CommunicationInterface::LEFT_MOTOR_ADRESS));
				stat.add<mrvk_driver::Mcb_status>("right board status", comunication_interface.getStatusMCB(CommunicationInterface::RIGHT_MOTOR_ADRESS));

			}else stat.summary(2, "Disconnected");
		}

		void statusTimerCallback(const ros::TimerEvent& timer_struct) {
			bool tmp = comunication_interface.getStatusMB().hardware_central_stop;
			if(!tmp && lastStopButtonVal){
				callResetCentralStopService();
			}
			lastStopButtonVal = tmp;
			diagnostic.update();
		}
		void callResetCentralStopService(){
			std_srvs::Trigger srv;
			if(!ros::service::call(n.getNamespace() + "/reset_central_stop",srv))
			ROS_ERROR("%s service not available",(n.getNamespace() + "reset_central_stop").c_str());
			if(!srv.response.success)
			ROS_WARN("Faild to reset central stop %s",srv.response.message.c_str());
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
	int rate;

	n.getParam("port_names",ports);
	n.param<int>("baudrate", baud, 230400);
	n.param<int>("stop_bits", stopBits, 1);
	n.param<int>("parity", parity, 1);
	n.param<int>("byte_size", byteSize, 8);
	n.param<int>("comunication_rate", rate, 30);

	Mrvk::Driver driver(ports,baud,stopBits,parity,byteSize);
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
