
#include <ros/ros.h>
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%

#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
//#include "mrvk/command.h"
//#include "mrvk/send_command_data.h"
#include <sensor_msgs/Joy.h>
#include <std_srvs/Trigger.h>
#include <ros/callback_queue.h>
// %EndTag(MSG_HEADER)%

#include <sstream>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <fstream>
#include <boost/make_shared.hpp>

#define VLAVO 0x44
#define VPRAVO 0x43
#define VPRED 0x41
#define VZAD 0x42
#define ESC 0x1B
#define BACKSPACE 0x4F

#define SHIFT_UP 0
#define SHIFT_DOWN 1
#define CAMERA_UP 2
#define CAMERA_DOWN 3
#define CAMERA_LEFT 4
#define CAMERA_RIGTH 5
#define MOTOR_STOP  6
#define CAMERA_STOP 7
#define GLOBAL_STOP 8
#define HELP 9
#define CAMERA_SOURCE 10

#define KROK_M_LINEAR 0
#define KROK_M_ANGULAR 1
#define KROK_K_LINEAR 2
#define KROK_K_ANGULAR 3




 // ros::Publisher radenie_pub;
  ros::Publisher cmd_vel_pub;
 // ros::Publisher cmd_vel_kamera_pub;
  ros::ServiceClient clientServiceMotor;
  ros::ServiceClient clientServiceMain;
 //  ros::ServiceClient clientServiceGlobalStop;
  // ros::ServiceClient clientServiceResetFlags;
    ros::ServiceClient clientServiceResetCentralStop;
    ros::ServiceClient clientServiceResetQ;
    ros::ServiceClient clientServiceArmVoltage;
   ros::ServiceClient clientServiceCameraSource;
   
   ros::ServiceClient clientServiceMotorSettings;
  ros::ServiceClient clientServiceMainSettings;
  ros::Subscriber joystick_subscriber;
   //ros::Subscriber tablet_subscriber;
   
void getch(char *c);
char getch_timeout0();
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
 void joystick_action(int tlacidlo);
//void radenie(int x);
void send_cmd_vel(char c);
void joy_send_cmd_vel(float angular, float linear);
//void send_camera_cmd_vel(int z,int x);
//void send_command(uint8_t data[],int write_length,int read_length);
bool send_settings(ros::ServiceClient& client);
void help();
void main_menu();
void main_board_menu();
void motor_board_menu();
void softver_menu();
int run();
int ukoncenie();
void keyboard_menu();
void joystick_menu();
 void joystick_test();
void zmen_klavesu(char* klavesa);
void zmen_tlacidlo(uint8_t *zmena);
 void oznacenie_osi(int os);
void clear_window();
void create_settings_files(int index);
void read_files();
void help_joystick();



 bool zabrzdene = 0;
 bool isRun = false;
 bool central_stop = true;
 bool zapisane_nastavenia_mb = true;
 bool zapisane_nastavenia_mcb = true;
 int nastavenia_krokov[4] = {50,25,10,10};
 bool cistenie_obrazovky = true;


 char keys[11] = {0x70,0x6c,0x77,0x73,0x61,0x64,0x20,0x72,0x74,0x68,0x71};

uint8_t joystick_buttons[11] = {8,9,22,23,20,21,0,5,1,6,4}; 

 int tlacidlo = 20;
 	
 ros::Time sendCommandTime;

 bool hasMotorSpeed = false;
 geometry_msgs::Twist msgTwist;
   char c[] = {0,0,0}; 



   //todo pokus o prerobenie do classy
 /*  class Keyboard{
   public:
   	Keyboard(boost::shared_ptr <ros::Publisher> cmd_vel_pub){

   		this->cmd_vel_pub = cmd_vel_pub;
   		stepSize[0] = 50; //motor
   		stepSize[1] = 25; //motor
   		stepSize[2] = 10; //kamera
   		stepSize[3] = 10; //kamera
   	}
   	void run(){

   		char c[3];
   		getch(c);

   		if ((c[0] == ESC) && (c[1] == 0x00) && (c[2] == 0x00)) //stlacena klavesnica ESC vyvola koniec programu
   				{
   				send_cmd_vel(0); //okamzite vynuti zastavenie
   				//send_camera_cmd_vel(0,0);
   				if (ukoncenie())
   				{
   				isRun = false;

   				}

   				if ((c[0] == 0x1B) && (c[1] == 0x5B)) //sipky, zadavanie rychlosti motoru
   					send_cmd_vel(c[2]);
   				else{
   					for (int i = 0; i < mappedKeys.size(); i++){
   						if (c[0] == mappedKeys[i].key){
   							callClient(mappedKeys[i].client);
   						}
   					}

   				}
   			}
   	}

   	void addServiceClient(boost::shared_ptr <ros::ServiceClient> client, char key){

   		MAPPED_KEYS map_key;
   		map_key.key = key;
   		map_key.client = client;

   		mappedKeys.push_back(map_key);
   	}

   	char getch_timeout0()
   	{
   		char buf = 0;
   	        struct termios old = {0};
   	        if (tcgetattr(0, &old) < 0)
   	                perror("tcsetattr()");
   	        old.c_lflag &= ~ICANON;
   	        old.c_lflag &= ~ECHO;
   	        old.c_cc[VMIN] = 1;
   	        old.c_cc[VTIME] = 0;
   	        if (tcsetattr(0, TCSANOW, &old) < 0)
   	                perror("tcsetattr ICANON");
   	        if (read(0, &buf, 1) < 0)
   	                perror ("read()");
   	        old.c_lflag |= ICANON;
   	        old.c_lflag |= ECHO;
   	        if (tcsetattr(0, TCSADRAIN, &old) < 0)
   	                perror ("tcsetattr ~ICANON");
   	        return (buf);
   	}
   private:
   	typedef struct mapped_keys{
   		char key;
   		boost::shared_ptr <ros::ServiceClient> client;
   	}MAPPED_KEYS;

   	boost::shared_ptr <ros::Publisher> cmd_vel_pub;
   	std::vector <MAPPED_KEYS> mappedKeys;

   	 int stepSize[4];

   	void getch(char *buf) {
   	        int i = 0;
   	        buf[0] = 0;
   	        buf[1] = 0;
   	        buf[2] = 0;
   	        struct termios old = {0};
   	        if (tcgetattr(0, &old) < 0)
   	                perror("tcsetattr()");
   	        old.c_lflag &= ~ICANON;
   	        old.c_lflag &= ~ECHO;
   	        old.c_cc[VMIN] = 0;
   	        old.c_cc[VTIME] = 1;
   	        if (tcsetattr(0, TCSANOW, &old) < 0)
   	                perror("tcsetattr ICANON");
   	        while (read(0, &buf[i++], 1) > 0);
   	                //perror ("read()");
   	        old.c_cc[VTIME] = 0;
   	        old.c_lflag |= ICANON;
   	        old.c_lflag |= ECHO;
   	        if (tcsetattr(0, TCSADRAIN, &old) < 0)
   	                perror ("tcsetattr ~ICANON");
   	}

   	bool callClient(boost::shared_ptr <ros::ServiceClient> client)
   	{
   	hasMotorSpeed = false;

   	std_srvs::Trigger srv;
   	if (client->call(srv))
   	{
   	  	if (srv.response.success)
   	  	{
   	  	printf("nastavenia boli zapisane\n");
   	  			 	//zapisane_nastavenia_mb = true;
   	  			 	return true;
   	  			 }
   	  			 else {
   	  			 	printf("zapis do zariadenia neuspesny\n error: %s\n",srv.response.message.c_str());
   	  			 	usleep(500000);
   	  			 }
   	  		} else {
   	  			printf("zlyhala komunikacia s nodou\n");
   	  			usleep(500000);
   	  		}
   	  		return false;
   	}

   	void send_cmd_vel(char c)
   	{

   		geometry_msgs::Twist msgTwist;

   			switch (c)
   			{	//rychlost sa deli 1000 preto aby sa upravil na rozsah motora, ktory je 0-1
   				case VLAVO : msgTwist.angular.z -= stepSize[KROK_M_ANGULAR]/1000.0;
   				break;
   				case VPRED : msgTwist.linear.x += stepSize[KROK_M_LINEAR]/1000.0;
   				break;
   				case VZAD : msgTwist.linear.x -=  stepSize[KROK_M_LINEAR]/1000.0;
   				break;
   				case VPRAVO : msgTwist.angular.z += stepSize[KROK_M_ANGULAR]/1000.0;
   				break;
   			}
   			if (msgTwist.linear.x > 1)
   				msgTwist.linear.x = 1;
   			if (msgTwist.linear.x < -1)
   				msgTwist.linear.x = -1;
   			if (msgTwist.angular.z > 1)
   				msgTwist.angular.z = 1;
   			if (msgTwist.angular.z < -1)
   				msgTwist.angular.z = -1;

   			cmd_vel_pub->publish(msgTwist);
   	}
   };
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "robot_control");

  ros::NodeHandle n;
 //  clientServiceMotor = n.serviceClient<mrvk::send_command_data>("test_motorov");
  // clientServiceMain = n.serviceClient<mrvk::send_command_data>("send_command");
  // clientServiceResetFlags = n.serviceClient<std_srvs::Trigger>("/reset_flags");
   //clientServiceGlobalStop = n.serviceClient<std_srvs::Trigger>("set_global_stop");
   clientServiceResetQ = n.serviceClient<std_srvs::Trigger>("reset_Q_batery");
   clientServiceArmVoltage = n.serviceClient<std_srvs::Trigger>("set_arm_voltage");
   clientServiceCameraSource = n.serviceClient<std_srvs::Trigger>("camera_source");
   clientServiceResetCentralStop = n.serviceClient<std_srvs::Trigger>("reset_central_stop");
    clientServiceMainSettings = n.serviceClient<std_srvs::Trigger>("write_main_board_settings");
    clientServiceMotorSettings = n.serviceClient<std_srvs::Trigger>("write_motor_boards_settings");
   cmd_vel_pub = n.advertise<geometry_msgs::Twist>("diff_drive_controller/cmd_vel", 1000);
  // cmd_vel_kamera_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_kamera", 1000);
  //radenie_pub = n.advertise<std_msgs::UInt8>("gear_shifting", 1000);
  joystick_subscriber = n.subscribe("/joy", 50, &joyCallback);
 // tablet_subscriber = n.subscribe("cmd_vel", 1, &tabletCallback);
  
  /*boost::shared_ptr <ros::Publisher> cmd_pub(&cmd_vel_pub);
  boost::shared_ptr <ros::ServiceClient> client_reset_central_stop(&clientServiceResetCentralStop);
  Keyboard keyboard(cmd_pub);
  keyboard.addServiceClient(client_reset_central_stop, keys[GLOBAL_STOP]);*/

  ros::Rate loop_rate(10);
  int x=0;
  int y=0;
//read_files();

  while (ros::ok() )
  {
	main_menu();	
	c[0] = getch_timeout0();
	if (c[0] == ESC)
	{
		if (ukoncenie())
			return 0;
			else continue;
	}	
	switch (c[0])
	{
		case '0' : run();
			 break; 
		case '1' : main_board_menu();
			 break;
		case '2' : motor_board_menu();
			 break;
		case '3' : softver_menu();
			break;	
		case '4' : help();
			getch_timeout0();
			break;	
		case '5' : help_joystick();
			getch_timeout0();
			break;	
	}

  }
  return 0;
}

int run()
{
	help();
	printf("\nprogram bezi robot je ovladatelny\n");
	isRun = true;
	sendCommandTime = ros::Time::now();
	while (ros::ok())
	{
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));


		getch(c);
		//printf("%X\n",c[0]);
		if ((c[0] == ESC) && (c[1] == 0x00) && (c[2] == 0x00)) //stlacena klavesnica ESC vyvola koniec programu
		{
		send_cmd_vel(0); //okamzite vynuti zastavenie 
		//send_camera_cmd_vel(0,0);
			if (ukoncenie())
			{
			isRun = false;
			return 0;
			}
		}	
	
		if ((c[0] == 0x1B) && (c[1] == 0x5B)) //sipky, zadavanie rychlosti motoru 
			send_cmd_vel(c[2]);
		else 
		{
			for (int i = 0; i < sizeof(keys);i++)
				if (keys[i] == c[0])
				switch (i)
				{
				case SHIFT_UP: //radenie(1);
						break;
 				case SHIFT_DOWN: //radenie(-1);
 						break;
 				case MOTOR_STOP: send_cmd_vel(0);
 						break;
 				case GLOBAL_STOP: 
 					//	ros::param::get("/mrvk/central_stop",central_stop); //todo toto treba prerobit na jednu funnkciu aj v rvk driver
 					//	if (central_stop)
 							send_settings(clientServiceResetCentralStop);
 						//else
 						//	send_settings(clientServiceGlobalStop);
 						break;
 				case CAMERA_UP: //send_camera_cmd_vel(nastavenia_krokov[KROK_K_LINEAR],0);
 						break;
 				case CAMERA_DOWN: //send_camera_cmd_vel(-1 * nastavenia_krokov[KROK_K_LINEAR], 0);
 						break;
 				case CAMERA_LEFT: //send_camera_cmd_vel(0, -1 * nastavenia_krokov[KROK_K_ANGULAR]);
 						break;
 				case CAMERA_RIGTH: //send_camera_cmd_vel(0, nastavenia_krokov[KROK_K_ANGULAR]);
 						break;
 				case CAMERA_STOP: //send_camera_cmd_vel(0,0);
 						break;
 				case HELP: help();
 						break;
					case CAMERA_SOURCE: send_settings(clientServiceCameraSource);
 						break;
 					}
			}	
				if (hasMotorSpeed && (ros::Time::now() - sendCommandTime).toSec() > 0.5)
					{

						sendCommandTime = ros::Time::now();
						 cmd_vel_pub.publish(msgTwist);
					}
	}
	return -1;
}

int last_buttons[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	static bool cameraTriger = false;
	
	if (isRun)
	{
		
		if (((joy->axes[0] != 0) || (joy->axes[1] != 0)) && joy->buttons[0])	
		{			
			joy_send_cmd_vel(joy->axes[0], joy->axes[1]);
		}
		else joy_send_cmd_vel(0.0,0.0);
		
			for (tlacidlo = 0;tlacidlo < 12; tlacidlo++)
			{
				if (joy->buttons[tlacidlo] == 1) 
				{
					if (!last_buttons[tlacidlo])
					{
						joystick_action(tlacidlo);
						last_buttons[tlacidlo] = 1;
					}
				}
				else last_buttons[tlacidlo] = 0;
				
			}
			
		
	//	if (tlacidlo == 12)
//		{
				if (joy->axes[4] > 0){
					if (!cameraTriger)
					{
						joystick_action(20);
						cameraTriger = true;
					}
				}
					else if (joy->axes[4] < 0){
							if (!cameraTriger)
						{
							joystick_action(21);
							cameraTriger = true;
						}
					}
					else if (joy->axes[5] > 0){
							if (!cameraTriger)
						{
							joystick_action(22);
							cameraTriger = true;
						}
					}
					else if (joy->axes[5] < 0)
					{
							if (!cameraTriger)
						{
							joystick_action(23);
							cameraTriger = true;
						}
					}else cameraTriger = false;
	//}
		//	else cameraTriger = false;
			/*tlacidlo += 4;
			for (int i = 2; tlacidlo < 20;i++)
			{
				if (joy->axes[i] < 0) 	
					joystick_action(tlacidlo);
				else if  (joy->axes[i] > 0)
					joystick_action(tlacidlo + 1);
				tlacidlo += 2;
			}*/	
		}	
	
	else //pre nastavovanie v menu
	{
		for (tlacidlo = 0;tlacidlo < 12; tlacidlo++)
		{
			if (joy->buttons[tlacidlo] == 1)
				break;			
		}
		if (tlacidlo == 12)	
		{
			for (int i = 0; tlacidlo < 20;i++)
			{
				if (joy->axes[i] < 0) 
					break;	
				else if  (joy->axes[i] > 0)
				{
					tlacidlo++;
					break;
				}
				tlacidlo += 2;
			}	
		}
			if (tlacidlo < 12) printf("stlacene tlacidlo %d\n",tlacidlo);
			else if (tlacidlo < 20)	oznacenie_osi(tlacidlo);
	}
}

 void joystick_action(int tlacidlo)
 {
// if (tlacidlo == joystick_buttons[SHIFT_UP])
 //	radenie(1);
 //else if (tlacidlo == joystick_buttons[SHIFT_DOWN])
 	//radenie(-1);
//else if (tlacidlo == joystick_buttons[MOTOR_STOP])
	//send_cmd_vel(0);
 if (tlacidlo == joystick_buttons[GLOBAL_STOP])
 {
 	//ros::param::get("/mrvk/central_stop",central_stop); 	//todo toto prerobit na jednu funkciu aj v mrvk_dirver
 	//if (central_stop)
 		send_settings(clientServiceResetCentralStop);
 //	else
 	//	send_settings(clientServiceGlobalStop);
 }
 //else if (tlacidlo == joystick_buttons[CAMERA_UP])
 	//send_camera_cmd_vel(nastavenia_krokov[KROK_K_LINEAR],0);
// else if (tlacidlo == joystick_buttons[CAMERA_DOWN])
 //	send_camera_cmd_vel(-1 * nastavenia_krokov[KROK_K_LINEAR], 0);
 //else if (tlacidlo == joystick_buttons[CAMERA_LEFT])
 	//send_camera_cmd_vel(0, -1 * nastavenia_krokov[KROK_K_ANGULAR]);
//else if (tlacidlo == joystick_buttons[CAMERA_RIGTH])
 //	send_camera_cmd_vel(0, nastavenia_krokov[KROK_K_ANGULAR]);
 //else if (tlacidlo == joystick_buttons[CAMERA_STOP])
 	//send_camera_cmd_vel(0,0);
 else if (tlacidlo == joystick_buttons[HELP])
 	help_joystick();
else if (tlacidlo == joystick_buttons[CAMERA_SOURCE])
  send_settings(clientServiceCameraSource);
}
/*
void tabletCallback(const geometry_msgs::Twist::ConstPtr& cmd)
{
	if  (!zabrzdene)
	{
		msgTwist.angular.z = cmd->angular.z;
		msgTwist.linear.x = cmd->linear.x;
		if (msgTwist.linear.x > 1)
			msgTwist.linear.x = 1;
		if (msgTwist.linear.x < -1)
			msgTwist.linear.x = -1;
		if (msgTwist.angular.z > 1)
			msgTwist.angular.z = 1;
		if (msgTwist.angular.z < -1)
			msgTwist.angular.z = -1;
		printf("\nziadana rychlost je %.4f\nziadany uhol je %.4f\n", msgTwist.linear.x, msgTwist.angular.z);
		if (fabsf(msgTwist.linear.x) < 0.15) || fabsf(msgTwist.angular.z) < 0.15)){
			msgTwist.linear.x = 0;
			msgTwist.angular.z = 0;
		}
		cmd_vel_pub.publish(msgTwist);
		
		if ((msgTwist.linear.x != 0) || (msgTwist.angular.z != 0))
		{
			sendCommandTime = ros::Time::now();
			hasMotorSpeed = true;
			}
			else hasMotorSpeed = false;
	}
	else if (zabrzdene) printf("zablokovane pre odblokvanie stlac medzernik\n");
}*/
/*
void radenie(int x)
{
	static std_msgs::UInt8 prevod;
	if ((x>0) && (prevod.data < 3))
	{
		prevod.data++;
		radenie_pub.publish(prevod);	
	}
	else if  ((x<0) && (prevod.data > 0))
	{
		prevod.data--;
		radenie_pub.publish(prevod);
	}
	printf("\nzaradeny prevod je %d\n",prevod.data);
}
*/
void send_cmd_vel(char c)
{
	
	
	if (c == 0) // ak by bola poziadavka okamziteho brzdenia
	{
		msgTwist.angular.z = 0;
		msgTwist.linear.x = 0;

		if (!zabrzdene) //ak nie je zabrzdene tak zabrzdi
		{
			printf("\npohyb robota je zablokovany\n");
			zabrzdene  = true;
			cmd_vel_pub.publish(msgTwist);
			hasMotorSpeed = false;
		}
		else 
		{			
			zabrzdene = false;	//ak bolo zabrzdene tak uvolni
			printf("\npohyb robota je umozneni\n");
		}
	}
	else if (!zabrzdene)
	{
		switch (c)
		{	//rychlost sa deli 1000 preto aby sa upravil na rozsah motora, ktory je 0-1
			case VLAVO : msgTwist.angular.z -= nastavenia_krokov[KROK_M_ANGULAR]/1000.0;
			break;
			case VPRED : msgTwist.linear.x += nastavenia_krokov[KROK_M_LINEAR]/1000.0;
			break;
			case VZAD : msgTwist.linear.x -=  nastavenia_krokov[KROK_M_LINEAR]/1000.0;
			break;
			case VPRAVO : msgTwist.angular.z += nastavenia_krokov[KROK_M_ANGULAR]/1000.0;
			break;
		}
		if (msgTwist.linear.x > 1)
			msgTwist.linear.x = 1;
		if (msgTwist.linear.x < -1)
			msgTwist.linear.x = -1;
		if (msgTwist.angular.z > 1)
			msgTwist.angular.z = 1;
		if (msgTwist.angular.z < -1)
			msgTwist.angular.z = -1;
		printf("\nziadana rychlost je %.4f\nziadany uhol je %.4f\n", msgTwist.linear.x, msgTwist.angular.z);
		cmd_vel_pub.publish(msgTwist);
		
		if ((msgTwist.linear.x != 0) || (msgTwist.angular.z != 0))
		{
			sendCommandTime = ros::Time::now();
			hasMotorSpeed = true;
			}
			else hasMotorSpeed = false;
	}
	else printf("zablokovane pre odblokvanie stlac medzernik\n");
}

void joy_send_cmd_vel(float angular,float linear)
{
	if ((angular == 0) && (linear == 0))
	{
		msgTwist.angular.z = 0;
		msgTwist.linear.x = 0;
		cmd_vel_pub.publish(msgTwist);
		 hasMotorSpeed = false;
		return;
	}
	if (((abs((angular  - msgTwist.angular.z) * 100)/100.0 >= 0.1)
		|| (abs((linear  - msgTwist.linear.x) * 100)/100.0 >= 0.1)) && (!zabrzdene))
	
	{
		msgTwist.angular.z = angular;
		msgTwist.linear.x = linear;
		if (msgTwist.linear.x > 1)
			msgTwist.linear.x = 1;
		if (msgTwist.linear.x < -1)
			msgTwist.linear.x = -1;
		if (msgTwist.angular.z > 1)
			msgTwist.angular.z = 1;
		if (msgTwist.angular.z < -1)
			msgTwist.angular.z = -1;
		printf("\nziadana rychlost je %.4f\nziadany uhol je %.4f\n", msgTwist.linear.x, msgTwist.angular.z);
		cmd_vel_pub.publish(msgTwist);
		if ((msgTwist.linear.x != 0) || (msgTwist.angular.z != 0))
		{
			sendCommandTime = ros::Time::now();
			hasMotorSpeed = true;
			}
			else hasMotorSpeed = false;
	}
	else if (zabrzdene) printf("zablokovane pre odblokvanie stlac medzernik\n");
}

/*
void send_camera_cmd_vel(int x,int z)
{
	static geometry_msgs::Twist msg;
	if ((x == 0) && (z == 0))
	{
		msg.angular.z = 0;
		msg.linear.x = 0;	
	}
	else{
		msg.angular.z += z;
		msg.linear.x += x;
		if (msg.angular.z > 180)
			msg.angular.z = 180;
		if (msg.angular.z < -120)
			msg.angular.z = -120;
		if (msg.linear.x > 90)
			msg.linear.x = 90;
		if (msg.linear.x < -30)
			msg.linear.x = -30;
	}
	printf("\notocenie kamery okolo x  %d\n  okolo z %d\n",(int) msg.linear.x,(int) msg.angular.z);
	cmd_vel_kamera_pub.publish(msg);	
}
*/


bool send_settings(ros::ServiceClient& client)
{
hasMotorSpeed = false;

std_srvs::Trigger srv;
if (client.call(srv))
{
  	if (srv.response.success)
  	{
  	printf("nastavenia boli zapisane\n");
  			 	//zapisane_nastavenia_mb = true;
  			 	return true;
  			 }
  			 else {
  			 	printf("zapis do zariadenia neuspesny\n error: %s\n",srv.response.message.c_str());
  			 	usleep(500000);
  			 }
  		} else {
  			printf("zlyhala komunikacia s nodou\n");
  			usleep(500000);
  		}
  		return false;	
}
void help()
{
	clear_window();
	printf("--------help--------\n\n");
	printf("zobrazenie helpu %c",keys[HELP]);
	printf("ovladanie motorov prostrednictvom sipiek, stlacanim sipiek sa rychlost motorov inkrementuje(linearny pohyb o %d, angularny pohyb %d), "
	,nastavenia_krokov[KROK_M_LINEAR],nastavenia_krokov[KROK_M_ANGULAR]);
	printf("to iste plati aj o uhle otacania sa. Nastavena je PID regulacia. Maximalny mozny rozsah je -1000 - 1000\n");
	printf("radenie je zabezpecene tlacidlami %c(hore) a %c(dole).\n",keys[SHIFT_UP],keys[SHIFT_DOWN]);
	if (keys[MOTOR_STOP] == 0x20)
	{
	printf("okamzite zabrzdenie sa vykona stlacenim medzernika, co nastavy vsetky rychlosti na 0. ");
	printf("ovladanie bude tak softverovo zablokovane odblokovanie je mozne opatovnim stlacenim medzernika.\n");
	}
	else 
	{
		printf("okamzite zabrzdenie sa vykona stlacenim klavesnice %c, co nastavy vsetky rychlosti na 0. ", keys[MOTOR_STOP]);
		printf("ovladanie bude tak softverovo zablokovane odblokovanie je mozne opatovnim stlacenim klavesnice %c.\n",keys[MOTOR_STOP]);
	}
	printf("ovladanie kamery %c (hore) %c (dole) %c (vlavo) %c (vpravo), ",keys[CAMERA_UP],keys[CAMERA_DOWN],keys[CAMERA_LEFT],keys[CAMERA_RIGTH]);
	printf("s linearnym krokom %d a angularnym krokom %d\n",nastavenia_krokov[KROK_K_LINEAR],nastavenia_krokov[KROK_K_ANGULAR]);
	printf("umiestenie kamery do home pozicie [0,0] %c\n",keys[CAMERA_STOP]);
	printf("vyvolanie central stopu  %c ak by bol central stop vyvolany tlacidlo sluzi na obnovu prevadzky\n",keys[GLOBAL_STOP]);
	printf("ukoncenie klavesnica ESC\n");
	
}

void help_joystick()
{
	clear_window();
	printf("--------help--------\n\n");
	printf("zobrazenie helpu %d",joystick_buttons[HELP]);
	printf("ovladanie motorov prostrednictvom zakladnej osi, do motorov sa posiela poziadavka na rychlost podla polohy joysticka. Nastavena je PID regulacia. Nato aby podvozok bol ovladatelny treba drzat tlacidlo 0\n");
	printf("radenie je zabezpecene tlacidlami %d(hore) a %d(dole).\n",joystick_buttons[SHIFT_UP],joystick_buttons[SHIFT_DOWN]);
	printf("ovladanie bude tak softverovo zablokovane odblokovanie je mozne opatovnim stlacenim klavesnice %d.\n",joystick_buttons[MOTOR_STOP]);	
	printf("ovladanie kamery vedlajsou osou");
	printf("s linearnym krokom %d a angularnym krokom %d\n",nastavenia_krokov[KROK_K_LINEAR],nastavenia_krokov[KROK_K_ANGULAR]);
	printf("umiestenie kamery do home pozicie [0,0] %d\n",joystick_buttons[CAMERA_STOP]);
	printf("vyvolanie central stopu  %d ak by bol central stop vyvolany tlacidlo sluzi na obnovu prevadzky \n",joystick_buttons[GLOBAL_STOP]);
	printf("ukoncenie klavesnica ESC\n");
	
}

void main_menu()
{
	clear_window();
	printf("--------main menu--------\n\n");
	printf("0: start\n");
	printf("1: Main board\n");
	printf("2: Motor board\n");
	printf("3: softver nastavania\n");
	printf("4: help\n");
	printf("5: help joystick\n");
	printf("ESC: koniec\n");
	
}

void main_board_menu()
{
	 ros::NodeHandle n;
//static MBCommand commandMB(0x03);
 if (!zapisane_nastavenia_mb)
 {
 	clear_window();
 	printf("existuju nastavenia, ktore este neboli zapisane do zariadenia\n");
 	sleep(1);
 }	
 while (1)
 {
 	
	clear_window();
	printf("\n--------main board menu--------\n1: nastavenia power managment MB\n2: nastavenie control MB\n3: nastavenie regulatora kamery\n");
	printf("4: odoslat nastavenia na MB\nESC koniec\n\n");
	c[0] = getch_timeout0();
	if (c[0] == ESC)
	{
		if (!zapisane_nastavenia_mb)
		{
			clear_window();
			printf("neboli odoslane ziadne nastavenia na zariadenie skoncit aj tak? stlac y\n");
			c[0] = getch_timeout0();
		if (c[0] == 0x79)
			break;
		}
		else break;
	}
	switch (c[0])
	{
	case '1': 
	
		while (1)
		{
			clear_window();
			printf("nastavenia power managment\n");
			printf("0: Napatie pre motor a senzor board 5V\n1: Napatie pre motor board 12V\n2: Videotransmitter\n3: Wifi\n");
			printf("4: Laser scanner\n5: GPS\n6: ARM 5V\n7:ARM 12V\n8: PC2\n9: Kamera\nESC: Spat\n\n");
			c[0] = getch_timeout0();
			if (c[0] == ESC)
				break;
			switch (c[0])
			{
				case '0':
				printf("MCBsSB 5V\n");
				c[0] = getch_timeout0();
				//commandMB.setMCBsSB_5V(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("mcb_5V", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '1':
				printf("mcb_12V\n");
				c[0] = getch_timeout0();
				//commandMB.setMCBs_12V(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("mcb_12V", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '2':
				printf("video transmitter\n");
				c[0] = getch_timeout0();
				//commandMB.setWifi(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("video_transmitter", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '3':
				printf("wifi\n");
				c[0] = getch_timeout0();
				//commandMB.setWifi(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("wifi", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '4':
				printf("laser scanner\n");
				c[0] = getch_timeout0();
				//commandMB.setLaser(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("laser_scanner", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '5':
				printf("gps\n");
				c[0] = getch_timeout0();
				//commandMB.setWifi(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("gps", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '6':
				printf("arm_5V\n");
				c[0] = getch_timeout0();
				//commandMB.setWifi(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("arm_5V", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '7':
				printf("arm_12V\n");
				c[0] = getch_timeout0();
				//commandMB.setWifi(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("arm_12V", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '8':
				printf("pc2\n");
				c[0] = getch_timeout0();
				//commandMB.setWifi(c[0] - 48);
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				n.setParam("pc2", (bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '9':
				printf("kamera\n");
				c[0] = getch_timeout0();
				//commandMB.setKamera(c[0] - 48);
				n.setParam("camera", (bool)(c[0] - 48));
				printf("zapisal som %d\n",(bool)(c[0] - 48));
				zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				default:
				printf("nenastavene dorobit program\n");	
			}
		}
		break;
	case '2':
		while(1)
		{
			clear_window();
			printf("\nnastavenie control\n");
			printf("0: Odblokovanie central stop\n1: Potvrdenie nabitych baterii a nulovanie pocitadla spotrebovaneho naboja");
			printf("\n2: zapnutie/vypnutie ramena\nESC: Spat\n\n");
			c[0] = getch_timeout0();
			if (c[0] == ESC)
				break;
			switch(c[0])
			{
				case '0':
				printf("Odblokovanie central stop\n");
				//c[0] = getch_timeout0();
				//commandMB.setCentralStop(c[0] - 48);
				send_settings(clientServiceResetCentralStop);
			//	printf("zapisal som %d\n",(bool)(c[0] - 48));
				//zapisane_nastavenia_mb = false;
				usleep(500000);
				break;
				case '1':
				printf("Potvrdenie nabitych baterii a nulovanie pocitadla spotrebovaneho naboja\n");
				//c[0] = getch_timeout0();
				//commandMB.setPowerOff(c[0] - 48);
				//printf("zapisal som %d\n",(bool)(c[0] - 48));
				//zapisane_nastavenia_mb = false;
				send_settings(clientServiceResetQ);
				usleep(500000);
				break;
				case '2':
				printf("zapnutie/vypnutie ramena\n");
				//c[0] = getch_timeout0();
				//commandMB.setPowerOff(c[0] - 48);
				//printf("zapisal som %d\n",(bool)(c[0] - 48));
				//zapisane_nastavenia_mb = false;
				send_settings(clientServiceArmVoltage);
				usleep(500000);
				break;	
			}
		}
		break;
	case '3':	
		clear_window();
		printf("nastavenie PID kamery rozsah 0-255\n");
		printf("\nProporcialna zlozka otacania\n");
		int pko;
		scanf("%d",&pko);
		printf("\nProporcialna zlozka klopenia\n");
		int pkk;
		scanf("%d",&pkk);
		printf("\nIntegralna zlozka otacania\n");
		int iko;
		scanf("%d",&iko);
		printf("\nIntegralna zlozka klopenia\n");
		int ikk;
		scanf("%d",&ikk);
		//commandMB.setKameraRegulator((uint8_t)pko,(uint8_t)pkk,(uint8_t)iko,(uint8_t)ikk);
		printf(" nastavenie na %d %d %d %d\n",pko,pkk,iko,ikk);
		n.setParam("kamera_PID_pko", pko);
		n.setParam("kamera_PID_pkk", pko);
		n.setParam("kamera_PID_iko", pko);
		n.setParam("kamera_PID_ikk", pko);
		zapisane_nastavenia_mb = false;
		usleep(500000);
		break;
	case '4':
		clear_window();
		//zapisane_nastavenia_mb = true;
		//uint8_t data[commandMB.controlCommandLength];
		//int read_length = commandMB.getControlCommand(data);
 		zapisane_nastavenia_mb = send_settings(clientServiceMainSettings);
 		//publish(data,21,read_length);
  		//send_command(data,commandMB.controlCommandLength,read_length);
  		
	}
  }
}

void motor_board_menu()
{ 

	 ros::NodeHandle n;
    if (!zapisane_nastavenia_mcb)
 {
 	clear_window();
 	printf("existuju nastavenia, ktore este neboli zapisane do zariadenia\n");
 	sleep(1);
 }	
 while (1)
 {
	clear_window();
	printf("--------motor board menu--------\n");
	printf("1: zadanie regulatora\n");	
	printf("2: prepnut regulaciu PID/PWM\n");
	printf("3: odoslat nastavenia na motor boardy\n");
	printf("ESC: spat\n");
	
	c[0] = getch_timeout0();
	if (c[0] == ESC)
	{
		if (!zapisane_nastavenia_mcb)
		{
			clear_window();
			printf("neboli odoslane ziadne nastavenia na zariadenie skoncit aj tak? stlac y\n");
			c[0] = getch_timeout0();
		if (c[0] == 0x79)
			break;
		}
		else break;
	}
	
	switch (c[0])
	{
 	case '1':
 		clear_window();
 		printf("nastavenie PID regulatora\n");
		printf("\nPH\n");
		int ph;
		scanf("%d",&ph);
		printf("\nPL\n");
		int pl;
		scanf("%d",&pl);
		printf("\nIH\n");
		int ih;
		scanf("%d",&ih);
		printf("\nIL\n");
		int il;
		scanf("%d",&il);
		
		printf(" nastavenie na %d %d %d %d\n",ph,pl,il,ih);
 	 	//lavy.setRegulatorPID((uint8_t)ph,(uint8_t)pl,(uint8_t)ih,(uint8_t)il);
 		//pravy.setRegulatorPID((uint8_t)ph,(uint8_t)pl,(uint8_t)ih,(uint8_t)il);
		n.setParam("motor_PID_ph", ph);
 		n.setParam("motor_PID_pl", pl);
		n.setParam("motor_PID_ih", ih);
		n.setParam("motor_PID_il", il);
 		zapisane_nastavenia_mcb = false;
 		usleep(500000);
 		break;
 	case '2':
 		clear_window();
 		printf("PID = 0, PWM = 1\n");
 	 	c[0] = getch_timeout0();
 	 	//lavy.setMotorControl(c[0] - 48);
 		//pravy.setMotorControl(c[0] - 48);
 		printf("zapisal som %d\n",(bool)(c[0] - 48));
 		n.setParam("pwm_control", (bool)(c[0] - 48));
 		zapisane_nastavenia_mcb = false;
 		usleep(500000);
 			break;
 	case '3':
		clear_window();
		//zapisane_nastavenia_mcb = true;
	   // read_length = lavy.getControlCommand(data);
	 //   pravy.getControlCommand(&data[lavy.controlCommandLength]);
 		//publish(data,21,read_length);
 		//printf("nastavenia boli zapisane\n");
  		//send_command(data,lavy.controlCommandLength*2,read_length);	
  		zapisane_nastavenia_mcb = send_settings(clientServiceMotorSettings);
 	} 
 }	 	
}

void softver_menu()
{
   while(1)
   {
	clear_window();
	printf("--------softverove nastavenia--------\n\n");
	printf("1: rozlozenie klavesnice\n");
	printf("2: rozlozenie tlacidiel na joysticku\n");
	printf("3: test rozlozenia tlacidiel na joysticku\n");
	printf("4: nastavenie velkosti kroku pre motor\n");
	printf("5: nastavenie velkosti kroku pre kameru\n");
	printf("6: zapnut/vypnut cistenie obrazovky\n");
	printf("ESC: spat\n");
	c[0] = getch_timeout0();
	if (c[0] == ESC)
		break;
	switch (c[0])
	{
	 case '1':
		keyboard_menu();
		create_settings_files(0);
  		break;
  	case '2':
  		joystick_menu();
  		create_settings_files(1);
  		break;
  	case '3':
  		joystick_test();
  		break;
  	case '4': 
		clear_window();
		printf("zadaj velkost kroku pre linearny pohyb\n");
		scanf("%d",&nastavenia_krokov[KROK_M_LINEAR]);
		printf("zadaj velkost kroku pre angularny pohyb\n");
		scanf("%d",&nastavenia_krokov[KROK_M_ANGULAR]);
		printf("nastavene\n");
		create_settings_files(2);
		usleep(500000);
		break;
	case '5': 
		clear_window();
		printf("zadaj velkost kroku pre linearny pohyb\n");
		scanf("%d",&nastavenia_krokov[KROK_K_LINEAR]);
		printf("zadaj velkost kroku pre angularny pohyb\n");
		scanf("%d",&nastavenia_krokov[KROK_K_ANGULAR]);
		printf("nastavene\n");
		create_settings_files(2);
		usleep(500000);
		break;
	case '6':
		clear_window();
		printf("1 - zapnute cistenie obrazovky, 0 - vypnute cistenie obrazovky\n");
		c[0] = getch_timeout0();
		cistenie_obrazovky = (bool)(c[0] - 48);
		create_settings_files(2);
		break;
	case '7':
		create_settings_files(1);
		break;
	}
   }
}

void keyboard_menu()
{
  while(1)
  {
  	clear_window();
	printf("stlac klavesnicu, ktoru chces menit\n\n");
	printf("zaradit hore: %c\n",keys[SHIFT_UP]);
	printf("zaradit dole: %c\n",keys[SHIFT_DOWN]);
	if (keys[MOTOR_STOP] == 0x20)
		printf("okamzite zabrzdenie: medzernik\n");
	else
		printf("okamzite zabrzdenie: %c\n",keys[MOTOR_STOP]);
	printf("kamera hore: %c\n",keys[CAMERA_UP]);
	printf("kamera dole: %c\n",keys[CAMERA_DOWN]);
	printf("kamera vlavo: %c\n",keys[CAMERA_LEFT]);
	printf("kamera vpravo: %c\n",keys[CAMERA_RIGTH]);
	printf("zobrazit help: %c\n",keys[HELP]);
	printf("zastavenie kamery: %c\n",keys[CAMERA_STOP]);
	printf("odpojenie motorov od napajania: %c\n",keys[GLOBAL_STOP]);
	printf("ESC: spat\n");
	c[0] = getch_timeout0();
	if (c[0] == ESC)
		break;
	printf("\nstlacena klavesnica %c",c[0]);
	for (int i = 0; i < sizeof(keys); i++)
		if(c[0] == keys[i])
			zmen_klavesu(&keys[i]);
  }
 }
 void joystick_menu()
{
  while(1)
  {
  	clear_window();
	printf("stlac tlacidlo, ktore chces menit\n\n");
	if (joystick_buttons[SHIFT_UP] >= 12) { printf("zaradit hore: "); oznacenie_osi(joystick_buttons[SHIFT_UP]);}
	else printf("zaradit hore: tlacidlo %d\n",joystick_buttons[SHIFT_UP]);
	if (joystick_buttons[SHIFT_DOWN] >= 12) { printf("zaradit dole: "); oznacenie_osi(joystick_buttons[SHIFT_DOWN]);}
	else printf("zaradit dole: tlacidlo %d\n",joystick_buttons[SHIFT_DOWN]);
	if (joystick_buttons[MOTOR_STOP] >= 12) { printf("okamzite zabrzdenie: "); oznacenie_osi(joystick_buttons[MOTOR_STOP]);}
	else printf("okamzite zabrzdenie: tlacidlo %d\n",joystick_buttons[MOTOR_STOP]);
	if (joystick_buttons[CAMERA_UP] >= 12) { printf("kamera hore: "); oznacenie_osi(joystick_buttons[CAMERA_UP]);}
	else printf("kamera hore: tlacidlo %d\n",joystick_buttons[CAMERA_UP]);
	if (joystick_buttons[CAMERA_DOWN] >= 12) { printf("kamera dole: "); oznacenie_osi(joystick_buttons[CAMERA_DOWN]);}
	else printf("kamera dole: tlacidlo %d\n",joystick_buttons[CAMERA_DOWN]);
	if (joystick_buttons[CAMERA_LEFT] >= 12) { printf("kamera vlavo: "); oznacenie_osi(joystick_buttons[CAMERA_LEFT]);}
	else printf("kamera vlavo: tlacidlo %d\n",joystick_buttons[CAMERA_LEFT]);
	if (joystick_buttons[CAMERA_RIGTH] >= 12) { printf("kamera vpravo: "); oznacenie_osi(joystick_buttons[CAMERA_RIGTH]);}
	else printf("kamera vpravo: tlacidlo %d\n",joystick_buttons[CAMERA_RIGTH]);
	if (joystick_buttons[HELP] >= 12) { printf("zobrazit help: "); oznacenie_osi(joystick_buttons[HELP]);}
	else printf("zobrazit help: tlacidlo %d\n",joystick_buttons[HELP]);
	if (joystick_buttons[CAMERA_STOP] >= 12) { printf("zastavenie kamery: "); oznacenie_osi(joystick_buttons[CAMERA_STOP]);}
	else printf("zastavenie kamery: tlacidlo %d\n",joystick_buttons[CAMERA_STOP]);
	if (joystick_buttons[GLOBAL_STOP] >= 12) { printf("odpojenie motorov od napajania: "); oznacenie_osi(joystick_buttons[GLOBAL_STOP]);}
	else printf("odpojenie motorov od napajania: tlacidlo %d\n",joystick_buttons[GLOBAL_STOP]);
	printf("ESC: spat\n");
	ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	getch(c);
	if (c[0] == ESC)
		break;
	if (tlacidlo < 20)
	{
		for (int i = 0; i < sizeof(joystick_buttons);i++)
			if (joystick_buttons[i] == tlacidlo)
				zmen_tlacidlo(&joystick_buttons[i]);
 	}
  }
 }
 void oznacenie_osi(int os)
 {
    switch (os)
    {
    case 12: printf("os 1 vpravo\n");
	     break;
    case 13: printf("os 1 vlavo\n");
	     break;
    case 14: printf("os 1 vzad\n");
	     break;
    case 15: printf("os 1 vpred\n");
	     break;
    case 16: printf("os 2 vpravo\n");
	     break;
    case 17: printf("os 2 vlavo\n");
	     break;
    case 18: printf("os 2 vzad\n");
	     break;
    case 19: printf("os 2 vpred\n");
	     break;
     }
}
 void joystick_test()
 {
 clear_window();
 printf("stlac tlacidlo, ktore chces menit\n");
	printf("ESC: spat\n");
  while(1)
  {
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
	getch(c);
	if (c[0] == ESC)
		break;
	if (tlacidlo < 20)
	{	if (tlacidlo == 12)
			printf("zmena uhlu otacania do prava\n");
		else if (tlacidlo == 13)
			printf("zmena uhlu otacania do lava\n");
		else if (tlacidlo == 14)
			printf("zmena rychlosti minus\n");
		else if (tlacidlo == 15)
			printf("zmena rychlosti plus\n");
		else if (tlacidlo == joystick_buttons[SHIFT_UP])
 			printf("zaradit hore\n");
 		else if (tlacidlo ==  joystick_buttons[SHIFT_DOWN])
 			printf("zaradit dole\n");
 		else if (tlacidlo ==  joystick_buttons[CAMERA_UP])
 			printf("kamera hore\n");
 		else if (tlacidlo == joystick_buttons[CAMERA_DOWN])
 			printf("kamera dole\n");
 		else if (tlacidlo == joystick_buttons[CAMERA_LEFT])
 			printf("kamera vlavo\n");
 		else if (tlacidlo == joystick_buttons[CAMERA_RIGTH])
 			printf("kamera vpravo\n");
 		else if (tlacidlo ==joystick_buttons[HELP])
 			printf("help\n");
 		else if (tlacidlo == joystick_buttons[CAMERA_STOP])
 			printf("zastavenie kamery\n");
 		else if (tlacidlo == joystick_buttons[MOTOR_STOP])
 			printf("zastavenie motorov\n");
 		else if (tlacidlo == joystick_buttons[GLOBAL_STOP])
 			printf("odpojenie od napajania\n");
 		else printf("tlacidlo nie je nastavene\n");
 	}
  }
 }
 
int ukoncenie()
{
	clear_window();
	printf("naozaj chces skoncit? stlac y\n");
	c[0] = getch_timeout0();
	if (c[0] == 0x79)
		return 1;
	else return 0;
}

void zmen_klavesu(char *klavesa)
{
	printf("vyber klavesnicu, na ktoru chces nastavit\n");
	c[0] = getch_timeout0();
	if (c[0] == ESC)
		return;
	for (int i = 0; i < sizeof(keys);i++)
		if (c[0] == keys[i])
		{
			printf("klavesnica je obsadena\n");
			return;
		}
	*klavesa = c[0];
	usleep(500000);
}
void zmen_tlacidlo(uint8_t *zmena)
{
	printf("vyber tlacidlo, na ktore chces nastavit\n");
	while(1)
	{
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
		getch(c);
		if (c[0] == ESC)
			return;
		if (tlacidlo < 20)
		{
			if ((tlacidlo <= 15) && (tlacidlo >= 12))
				printf("zakladne osi su rezervovane pre zakladny pohyb, vyber ine tlacidlo\n");
			else
			{
				int i = 0;
				for (;i < sizeof(joystick_buttons);i++)
					if (joystick_buttons[i] == tlacidlo)
					{
						printf("tlacidlo je obsadene\n");
						sleep(1);
						break;
					}
			 if (i == sizeof(joystick_buttons))
			 {*zmena = tlacidlo;
			 return;
			 }
				 
			 }
		}
	}
}
void create_settings_files(int index)
{
  std::ofstream myfile;
   switch(index)
   {
   	case 0:
   	myfile.open ("src/mrvk/keyboard.txt");
   	for (int i = 0; i < sizeof(keys);i++)
  		myfile << keys[i] << "\n";
  	myfile.close();
  	break;
  	case 1:
	myfile.open ("src/mrvk/joystick.txt");
	for (int i = 0; i < sizeof(joystick_buttons);i++)
  		myfile << (int)joystick_buttons[i] << "\n";
 	myfile.close();
	break;
	case 2:
	myfile.open("src/mrvk/nastavenia.txt");
	myfile << (int)cistenie_obrazovky << "\n";
	for (int i = 0; i < 4;i++)
  		myfile << nastavenia_krokov[i] << "\n";
 	myfile.close();
	break;
	}

}
void read_files()
{
std::string line;
int i = 0;
std::ifstream myfile("src/mrvk/keyboard.txt");
if (myfile.is_open())
  {
    char pom = 0;
    while ( getline (myfile,line))
    {
        sscanf(line.c_str(),"%c",&pom); 
        if (pom  != 0)
        	keys[i] = pom;
        i++;
    }
    myfile.close();
  }
  
myfile.open("src/mrvk/nastavenia.txt");
if (myfile.is_open())
{
   getline(myfile,line);
   int pom = 0;
   sscanf(line.c_str(),"%d",&pom);
   if (pom == 1)
  	cistenie_obrazovky = true;
  else cistenie_obrazovky = false;
   i = 0;
    while (getline(myfile,line))
    {
        if (sscanf(line.c_str(),"%d",&pom) == 1)
        	nastavenia_krokov[i] = pom;
        i++;
    }
    myfile.close();
  }

myfile.open("src/mrvk/joystick.txt");
if (myfile.is_open())
  {
     i = 0;
    int pom = 0;
    while ( getline (myfile,line))
    {
        if (sscanf(line.c_str(),"%d",&pom) == 1)
        	 joystick_buttons[i] = pom;
        i++;
    }
    myfile.close();
  }
}
void getch(char *buf) {
        int i = 0;
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 0;
        old.c_cc[VTIME] = 1;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        while (read(0, &buf[i++], 1) > 0);
                //perror ("read()");
        old.c_cc[VTIME] = 0;
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");      
}
char getch_timeout0()
{  
	char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}
void clear_window()
{
	if (cistenie_obrazovky)
	std::system("clear");
	else printf("\n\n");
}
