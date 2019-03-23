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
			diagnostic.add("main board status", this, &Mrvk::Driver::mainBoardDiagnostics);
			diagnostic.add("left motor board status", this, &Mrvk::Driver::leftMotorBoardDiagnostics);
			diagnostic.add("right motor board status", this, &Mrvk::Driver::rightMotorBoardDiagnostics);
			diagnostic.setHardwareID("");

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

		void mainBoardDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){
			if (comunication_interface.isActive()){
				stat.summary(0, "Connected");
				auto msg = comunication_interface.getStatusMB();

				stat.add<bool>("central_stop", msg.central_stop);
				stat.add<bool>("hardware_central_stop", msg.hardware_central_stop);
				stat.add<float>("temperature", msg.temperature);
				stat.add<bool>("power_off_sequence", msg.power_off_sequence);
				stat.add<bool>("full_battery", msg.full_battery);
				// battery
				stat.add<float>("charge", msg.battery.charge);
				stat.add<float>("battery1_voltage", msg.battery.battery1_voltage);
				stat.add<float>("battery2_voltage", msg.battery.battery2_voltage);
				stat.add<float>("current", msg.battery.current);
				//power management
                stat.add<bool>("MCBsSB_5V", msg.power_managment.MCBsSB_5V);
                stat.add<bool>("MCBs_12V", msg.power_managment.MCBs_12V);
                stat.add<bool>("videotransmitter", msg.power_managment.videotransmitter);
                stat.add<bool>("fan", msg.power_managment.fan);
                stat.add<bool>("laser", msg.power_managment.laser);
                stat.add<bool>("gps", msg.power_managment.gps);
                stat.add<bool>("arm_5V", msg.power_managment.arm_5V);
                stat.add<bool>("arm_12V", msg.power_managment.arm_12V);
                stat.add<bool>("pc2", msg.power_managment.pc2);
                stat.add<bool>("camera", msg.power_managment.camera);
			} else {
				stat.summary(2, "Disconnected");
			}
		}

		void leftMotorBoardDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){
			if (comunication_interface.isActive()){
				stat.summary(0, "Connected");
				auto msg = comunication_interface.getStatusMCB(CommunicationInterface::LEFT_MOTOR_ADRESS);

				stat.add<std::string>("motor_name", msg.motor_name);
				stat.add<bool>("i2c_communication_error", msg.i2c_communication_error);
				stat.add<bool>("i2c_error_status", msg.i2c_error_status);
				stat.add<bool>("rs422_communication_error", msg.rs422_communication_error);
				stat.add<bool>("shifting", msg.shifting);
				stat.add<bool>("shifting_error", msg.shifting_error);
				stat.add<int>("gear_position", msg.gear_position);
				stat.add<float>("gear_current", msg.gear_current);
				stat.add<float>("board_voltage", msg.board_voltage);
				stat.add<float>("motor_voltage", msg.motor_voltage);
				stat.add<float>("motor_current", msg.motor_current);
			} else {
				stat.summary(2, "Disconnected");
			}
		}

		void rightMotorBoardDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat){

			if (comunication_interface.isActive()){
				stat.summary(0, "Connected");
				auto msg = comunication_interface.getStatusMCB(CommunicationInterface::RIGHT_MOTOR_ADRESS);

				stat.add<std::string>("motor_name", msg.motor_name);
				stat.add<bool>("i2c_communication_error", msg.i2c_communication_error);
				stat.add<bool>("i2c_error_status", msg.i2c_error_status);
				stat.add<bool>("rs422_communication_error", msg.rs422_communication_error);
				stat.add<bool>("shifting", msg.shifting);
				stat.add<bool>("shifting_error", msg.shifting_error);
				stat.add<int>("gear_position", msg.gear_position);
				stat.add<float>("gear_current", msg.gear_current);
				stat.add<float>("board_voltage", msg.board_voltage);
				stat.add<float>("motor_voltage", msg.motor_voltage);
				stat.add<float>("motor_current", msg.motor_current);
			} else {
				stat.summary(2, "Disconnected");
			}
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
