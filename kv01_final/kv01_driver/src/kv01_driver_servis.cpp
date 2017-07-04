#include <kv01_driver/driver.h>
#include <kv01_driver/serial_data.h>
#include "kv01_driver/common_functions.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int8.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include "kv01_driver/joint_speed.h"
#include <serial/serial.h>

serial::Serial* my_serial; // zamutexovane
common_functions* kv01_fun; //netreba mutexovat
ros::Duration control(0,500000000); //zamutexovane
bool menu_run = false;
bool wifi_fail = false;
double global_speed[6]={0,0,0,0,0,0};//zamutexovane
double position[6],real_position[6]; //zamutexovane
int poloha_mimo[6] ={0, 0, 0, 0, 0, 0};// mutex
bool flag_position_setup[6] = {false,false,false,false,false,false};//zamutexovane

ros::ServiceClient reset_voltage;
std_srvs::SetBool reset_voltage_data;  //netreba mutex


bool serial_comm(uint8_t *wBuffer,uint8_t *response)
{
    int bytes_read ,bytes_wrote;
    int message_length=0;
    uint8_t *rBuffer = new uint8_t[15];
    uint8_t *byte = new uint8_t[1];
    
  /* if(wBuffer[0]==0xe6){
      return true;
    }*/
    
    bytes_wrote =my_serial->write(wBuffer,(int)wBuffer[1]);
    
    if (bytes_wrote != wBuffer[1])
    {
      ROS_ERROR("Správa sa korektne nezapísala");
      my_serial->flushInput();
      delete [] rBuffer;
      return false; 
    }
    //cakanie kym sa nieco objavi na  vstupnom bufferi ak ano zacneme to citat precitam vsetko max 15 bytov
    ros::Time begin = ros::Time::now();
    
    while(ros::ok())
    {
      if (my_serial->available() > 0)
      {
	bytes_read = my_serial->read(byte,1);
	rBuffer[message_length]=byte[0];
	if (message_length >= 2)
	{
	  if (message_length > rBuffer[1])
	  break;   
	} 
	message_length++;
	if(byte[0]== 0xf0)
	break;	
      }
      else if ((ros::Time::now() - begin) > ros::Duration(2,0))
      {
	ROS_ERROR("Logicka cast vypnuta zapinam napajanie");
	reset_voltage_data.request.data = true;
	if (reset_voltage.call(reset_voltage_data))
	{  
	 ROS_ERROR("Napajanie zapnute");
	 usleep(100000);
	 delete [] rBuffer;
	 return false;
	}
	else 
	{
	  ROS_ERROR("Neuspesny pokus o zapnutie napajania skontrolujte hlavny pocitac");
	  ros::shutdown();  
	} 
      }
    }  
   // ROS_INFO("citam");
    // po precitani najdeme koniec spravy a zaznamename si jej dlzku
    
    if (rBuffer[1] == UC_data1 && rBuffer[2] == UC_data2){
      ROS_ERROR("UNKNOW COMMAND");
      delete [] rBuffer;
      return false;
    }
    
    if (rBuffer[1] == TO_data1 && rBuffer[2] == TO_data2)
    {
      ROS_ERROR("Time out komunikacie: Resetujem napajanie");
      reset_voltage_data.request.data = true;
      if (reset_voltage.call(reset_voltage_data))
      {  
	ROS_ERROR("Napajanie Vypnute");
      }
      else 
      {
	ROS_ERROR("Neuspesny reset napajania vypinam driver");
	ros::shutdown();  
      } 
      usleep(100000);
      ROS_ERROR("Time out komunikacie: Resetujem napajanie");
      reset_voltage_data.request.data = false;
      if (reset_voltage.call(reset_voltage_data))
      {  
	ROS_ERROR("Napajanie opatovne zapnute");
      }
      else 
      {
	ROS_ERROR("Neuspesny reset napajanie vypinam driver");
	ros::shutdown();  
      }
      usleep(100000);
	
     }
    // ak nic nie je na vstupnom buffery odpoved od menica je rovna tomu ktoremu sme posielali a dlzka bufera je spravna mozme podlsat response o spravnej odpovedi
   
   if(wBuffer[2]==0xD1){
   for(int i=0;i<message_length;i++){
   
     ROS_INFO("%X",rBuffer[i]);
   }
}
   if(wBuffer[2]==0xD5){
   for(int i=0;i<message_length;i++){
   
     ROS_INFO("%X",rBuffer[i]);
   }
}
   if ((my_serial->available() == 0) && (wBuffer[0] == rBuffer[0]) && (rBuffer[1] == (message_length)))
    {
    
      for (int i = 0; i < message_length; i++) 
      {
	response[i]= rBuffer[i];
	//ROS_INFO("%X",response[i]);
      }
      delete [] byte;
      delete [] rBuffer;
      return true;
    }
    //V pripade ze to neplati niekde je chyba vraciame false a vypis chyby pri citani
    else
    {
      ROS_ERROR("Chyba pri Citani");
      my_serial->flushInput();
      delete [] byte;
      delete [] rBuffer;
      return false;
    }// Na zaver precistime buffery
}


void joint_speed_callback(const kv01_driver::joint_speed::ConstPtr& speed)
{
    // joint hovori o prikaze pre gripper v pripade 1 otvor v pripade 2 zavri v pripade 3 stoj   
    pthread_mutex_lock( &wifi_mutex);
    if(wifi_fail && speed->speed != 0){
      pthread_mutex_unlock( &wifi_mutex);
      return;
    }
    else
    pthread_mutex_unlock( &wifi_mutex);
    
    uint8_t *message;
    uint8_t *rBuffer =new uint8_t[15];
    bool motor_danger;
    
    if (speed->command == comm_gripper_control)
    {
      message = kv01_fun->ArmCreateD5(message, speed->joint);
      for(int i=0;i<6;i++){
	ROS_INFO("%x",message[i]);
      }
      pthread_mutex_lock( &comm_lock_mutex);
      if (!serial_comm(message,rBuffer))
      {
	ROS_ERROR("Chyba v sprave pre gripper");
      }
      pthread_mutex_unlock( &comm_lock_mutex);
      delete [] rBuffer;
      return;
    }

    pthread_mutex_lock( &poloha_mimo_mutex);
    
    if(poloha_mimo[speed->joint] == -1 && speed->speed < 0)
    {
      ROS_ERROR("KLB %d mimo pracovnu polohu",speed->joint);
      pthread_mutex_unlock( &poloha_mimo_mutex);
      delete [] rBuffer;
      return;
    }
    else if (poloha_mimo[speed->joint] == 1 && speed->speed > 0)
    {
      ROS_ERROR("KLB %d mimo pracovnu polohu",speed->joint);
      pthread_mutex_unlock( &poloha_mimo_mutex);
      delete [] rBuffer;
      return;
    }
    else if(poloha_mimo[speed->joint] == -1 && speed->speed > 0 || poloha_mimo[speed->joint] == 1 && speed->speed < 0)
    {
      poloha_mimo[speed->joint]=0;
    }
    
    pthread_mutex_unlock( &poloha_mimo_mutex);
    
    pthread_mutex_lock( &brzdy_mutex);
    motor_danger = kv01_fun->odbrzdene[speed->joint];
    pthread_mutex_unlock(&brzdy_mutex);
    
    switch(speed->command)
    {
      case comm_position:
	  {
	    if(!motor_danger)
	    {
	      ROS_ERROR("Motor na klbe %d je zabrzdeny!",speed->joint);
	      delete [] rBuffer;
	      return;
	    }
	    else
	    {
	      message=kv01_fun->ArmCreateD0(comm_motory_speed, speed->joint, speed->speed);
	      
	      pthread_mutex_lock( &poss_lock_mutex);
	      position[speed->joint]=speed->position;
	      pthread_mutex_unlock( &poss_lock_mutex);
	      //ROS_INFO("SERVIS KLB:%d SPEED:%lf POZICIA %lf",req.joint,req.value, req.position);
	    }
	    break;
	  }
      case comm_motory_speed:
	  {
	    if(!motor_danger )
	    {
	      ROS_ERROR("Motor na klbe %d je zabrzdeny!",speed->joint);
	      delete [] rBuffer;
	      return;
	    }
	    else
	    {
	      message=kv01_fun->ArmCreateD0(comm_motory_speed, speed->joint, speed->speed);
	      break;
	    }
	  }   	  
    }
    
    pthread_mutex_lock( &comm_lock_mutex);
    
    if (!serial_comm(message,rBuffer)){
      ROS_ERROR("Vážna chyba v komunikácii vypínnam driver");
      ros::shutdown();
    }
    
    pthread_mutex_unlock( &comm_lock_mutex);
   
    switch(speed->command)
    {
      case comm_motory_speed:
	    {     
	      pthread_mutex_lock( &speed_mutex);
              global_speed[speed->joint]=speed->speed;
              pthread_mutex_unlock( &speed_mutex);
	      break;
	    }
	    case comm_position:
	    {
	      pthread_mutex_lock( &speed_mutex);
              global_speed[speed->joint]=speed->speed;
              pthread_mutex_unlock( &speed_mutex);
	      
	      pthread_mutex_lock( &flag_mutex);
	      flag_position_setup[speed->joint]=true;
	      pthread_mutex_unlock( &flag_mutex);
	      break;
	    }
    }
    delete [] rBuffer;
    return;
  
  
} 
void *position_control_thread(void *node)
{ 
 ros::NodeHandle *n=(ros::NodeHandle *) node;
 ros::Publisher speed_pub = n->advertise<kv01_driver::joint_speed>("/kv01/joint_speed", 12);
 kv01_driver::joint_speed joint_speed;
 ros::Rate loop_rate(50);
 
 std::string joint_string[6]={"0","1","2","3","4","5"};
 double speed[6];
 double real_position_in_fun[6];
 
 while(ros::ok())
 {

    for(int i=0;i<6;i++)
    {
      pthread_mutex_lock( &real_poss_lock_mutex);
      real_position_in_fun[i]= real_position[i];
      pthread_mutex_unlock( &real_poss_lock_mutex);
      
      pthread_mutex_lock( &speed_mutex);
      speed[i]=global_speed[i];
      pthread_mutex_unlock( &speed_mutex);
      
      if (((speed[i]>0) && (real_position_in_fun[i] > (kv01_fun->machine_data.Ulimit_j[i]-0.17) )) || ((speed[i]<0) && (real_position_in_fun[i] < (kv01_fun->machine_data.Dlimit_j[i]+0.17) )))
      {
	
	
	pthread_mutex_lock( &poloha_mimo_mutex);
	if (speed[i] > 0)
	  poloha_mimo[i] = 1;
	else if (speed[i] < 0)
	  poloha_mimo[i] = -1;
	pthread_mutex_unlock( &poloha_mimo_mutex);
	
	joint_speed.joint = i;
	joint_speed.command = comm_motory_speed;
	joint_speed.speed = 0;
        speed_pub.publish(joint_speed);
	ROS_ERROR("KLB %d mimo pracovnu polohu",i);  
	
      }
      pthread_mutex_lock( &flag_mutex);
      if (flag_position_setup[i])
      {
	pthread_mutex_lock( &poss_lock_mutex);
	//
	//ROS_INFO("VLAKNO KLB:%d SPEED:%lf POZICIA %lf realna pozicia %lf",i,speed[i], position[i],real_position_in_fun[i]);
	if ((((speed[i]>0) && (real_position_in_fun[i] > position[i])) || ((speed[i]<0) && (real_position_in_fun[i] < position[i])) || speed[i] == 0))
	{
	  joint_speed.joint = i;
	  joint_speed.command = comm_motory_speed;
	  joint_speed.speed = 0;
	  speed_pub.publish(joint_speed);
	  flag_position_setup[i]=false;
	}
	pthread_mutex_unlock( &poss_lock_mutex);
      }
      pthread_mutex_unlock( &flag_mutex);
      
     } 
     loop_rate.sleep();
  }
}

void *live_wifi_thread(void *node)
{ 
  ros::NodeHandle *n=(ros::NodeHandle *) node;
  ros::Publisher speed_pub = n->advertise<kv01_driver::joint_speed>("/kv01/joint_speed", 12);
  kv01_driver::joint_speed joint_speed;
  
  kv01_driver::driver driver_client_data; 
  ros::ServiceClient driver_client = n->serviceClient<kv01_driver::driver>("/kv01/driver_servis");
  
  ros::Duration minus_control(0,150000000);
  ros::Rate loop_rate(5);
  
  while(ros::ok())
  {
    pthread_mutex_lock( &wifi_mutex);
    if(menu_run == true && wifi_fail == false)
      control = control - minus_control;
    
    
   // ROS_INFO("Cas sec:%d nsec:%d", control.sec,control.nsec);
    if(wifi_fail)
      ROS_ERROR("Restartujte komunikaciu padla wifi robot je zabrzdeny");
   
    if (control <= ros::Duration(0,0) && wifi_fail == false)
    {  
      
      pthread_mutex_unlock( &wifi_mutex);
      for(int i=0;i <6; i++)
      {
	joint_speed.joint = i;
	joint_speed.command = comm_motory_speed;
	joint_speed.speed = 0;
	speed_pub.publish(joint_speed);
      }
      usleep(200000);
      int joint_message[6]={1,1,1,1,1,0};
      
      for (int i =0;i<6;i++)
      {
	  driver_client_data.request.joint[i]=joint_message[i]; 
      }

      driver_client_data.request.command=comm_brzdy;
      driver_client_data.request.data=comm_brzdy_stop;
      driver_client_data.request.value=0;
      driver_client_data.request.position= 0;
      
      if (driver_client.call(driver_client_data))
      {  
	 ROS_ERROR("WIFI fail: Brzdy zabrzdene");
      }
      else
      {
	pthread_mutex_unlock( &wifi_mutex);
	ROS_INFO("WIFI fail:Neuspesne zabrzdenie vypinam driver");
	ros::shutdown();
      } 
      
      driver_client_data.request.joint[5]=1; 
      driver_client_data.request.command=comm_message;
      driver_client_data.request.data=comm_motory_dis;
      driver_client_data.request.value=0;
      driver_client_data.request.position = 0;

      if (driver_client.call(driver_client_data))
      {	
	 ROS_ERROR("WIFI fail: Motory vypnute");
      }
      else
      {
	ROS_INFO("WIFI fail:Neuspesne vypnutie motorov vypinam driver");
	ros::shutdown();
      }  
      wifi_fail = true;
     }
     else
     pthread_mutex_unlock( &wifi_mutex);
    loop_rate.sleep();
  }
}

void *joint_states_fun_thread(void *node)
{
  ros::Rate loop_rate(50);//finalna frekvencia komunikacie cela vzorka prida raz za 80 ms pricom menic komunikuje s pc za cca 13 ms
  
  uint8_t *message;
  uint8_t *rBuffer = new uint8_t[15];
  ros::NodeHandle *n=(ros::NodeHandle *) node;
  ros::Publisher joint_pub = n->advertise<sensor_msgs::JointState>("/kv01/joint_states",6);

  sensor_msgs::JointState joint_state;
  
  joint_state.header.frame_id = "kv01_states";
  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] ="klb0";
  joint_state.name[1] ="klb1";
  joint_state.name[2] ="klb2";
  joint_state.name[3] ="klb3";
  joint_state.name[4] ="klb4";
  joint_state.name[5] ="klb5";
  sleep(2);
  
  while(ros::ok())
  { 
    for(int joint=0;joint<6;joint++)
    {  
      message=kv01_fun->ArmCreateD3(message,joint);
      
      pthread_mutex_lock( &comm_lock_mutex);
      if (!serial_comm(message,rBuffer))
      {
        ROS_ERROR("Unknow command alebo Timeout pre citanie pozicie");
	pthread_mutex_unlock( &comm_lock_mutex);
	continue;
      }
      pthread_mutex_unlock( &comm_lock_mutex);
     // v prípade -10 je error pri citani
     joint_state.position[joint]=kv01_fun->ArmReceiveD3(rBuffer,joint);
     pthread_mutex_lock( &real_poss_lock_mutex);
     real_position[joint]=joint_state.position[joint];
     pthread_mutex_unlock( &real_poss_lock_mutex);
     
     }
     joint_state.header.stamp = ros::Time::now();  
     joint_pub.publish(joint_state);
     loop_rate.sleep();
  }
  delete [] rBuffer;
  return 0; 
  
  
}

void wifi_live(const std_msgs::Int8::ConstPtr& msg)
{ 
  
  if (msg->data == 1)
  {
    pthread_mutex_lock( &wifi_mutex);
    control=ros::Duration(0,500000000); 
    if(!menu_run)
      menu_run=true;
    pthread_mutex_unlock( &wifi_mutex);
  }
return;
}

bool WriteAndRead(kv01_driver::driver::Request  &req,
		  kv01_driver::driver::Response &res)
   {
    pthread_mutex_lock( &wifi_mutex);
    
    if(wifi_fail){
      ROS_ERROR("Padnuta wifi");
      pthread_mutex_unlock( &wifi_mutex);
      return false;
    }
    else
    pthread_mutex_unlock( &wifi_mutex);
    kv01_driver::serial_data serial_srv; 
    bool motor_danger;
    bool serv_ok=false;
    uint8_t *message;
    
    for(int xjoint=0;xjoint<6;xjoint++)
    {
      if((int)req.joint[xjoint] != 1)
	continue;
    uint8_t *message;
    uint8_t *rBuffer =new uint8_t[15];
    
    ROS_INFO("cyklus %d",xjoint); 
    std::string joint_string = kv01_fun->to_string(xjoint);

      
      switch(req.command)
      {
	case comm_voltages:
	{
	  message=kv01_fun->ArmCreateD4(message,xjoint);
	  break;
	}
	case comm_brzdy:
	{
	  double speed;
	  
	  pthread_mutex_lock( &speed_mutex);
          speed=global_speed[xjoint];
          pthread_mutex_unlock( &speed_mutex);
	  
	  if (req.data == comm_brzdy_stop && speed != 0)
	  {
	    ROS_ERROR("Motor sa hybe");
	    delete [] rBuffer;
	    return false;
	  }
	  else
	    message=kv01_fun->ArmCreateD1(message,xjoint,req.data); 
	  break;
	}
	
	case comm_message:
	{
	  switch(req.data)
	  {
	    case comm_motory_en:
	    {
	      message=kv01_fun->ArmCreateD0(req.data, xjoint, 0);
	      break;
	    }
	    case comm_motory_dis:
	    {
	      message=kv01_fun->ArmCreateD0(req.data, xjoint, 0);
	      break;
	    }
	  }
	  break;
	}
	case comm_status:
	{
	  message=kv01_fun->ArmCreateD0(comm_status, xjoint, 0);
	  break;
	}
	default:
	{ 
	  res.error=comm_err_command;
	  ROS_ERROR("Chybny prikaz");
	  delete [] rBuffer;
	  return false;
	}
      }
      
      pthread_mutex_lock( &comm_lock_mutex);
      
      if (!serial_comm(message,rBuffer))
      {
	delete [] rBuffer;
        return false;
      }
 
      pthread_mutex_unlock( &comm_lock_mutex);
      
      switch(req.command)
      {
	case comm_message:
	{
	  if (kv01_fun->check_response(rBuffer) == true)
	  {
	    switch(req.data)
	    {
	      case comm_motory_en:
	      {
		ros::param::set("/kv01/motorklb"+joint_string,true);
		serv_ok=true;
		break;
	      }
	      case comm_motory_dis:
	      {
		ros::param::set("/kv01/motorklb"+joint_string,false); 
		serv_ok=true;
		break;
	      }
	    }
	  }
	  else {
	    delete [] rBuffer;
	    return false;
	  }
	}
	break;
	case comm_status:
	{ 
	  pthread_mutex_lock( &speed_mutex);
	  ros::param::set("/kv01/speedklb"+joint_string,global_speed[xjoint]);
          pthread_mutex_unlock( &speed_mutex); 
	  
	  pthread_mutex_lock( &brzdy_mutex);
	  ros::param::set("/kv01/odbrzdeneklb"+joint_string,kv01_fun->odbrzdene[xjoint]);
          pthread_mutex_unlock(&brzdy_mutex);
	  
	  if (kv01_fun->ArmReceiveD0(rBuffer,xjoint,comm_status))
	  {
	    serv_ok=true;
	  }
	  else{
	    delete [] rBuffer;
	    return false;
	  }
	  break;
	}
	
	case comm_voltages:
	{
	  kv01_fun->ArmReceiveD4(rBuffer); 
	  ros::param::set("/kv01/volt5V_jklb"+joint_string,kv01_fun->rob_arm_voltages.volt5V_j);
	  ros::param::set("/kv01/volt12V_jklb"+joint_string,kv01_fun->rob_arm_voltages.volt12V_j);
	  ros::param::set("/kv01/volt24V_jklb"+joint_string,kv01_fun->rob_arm_voltages.volt24V_j);
	  serv_ok=true;
	  break;
	}
	
	case comm_brzdy:
	{
	  if (kv01_fun->ArmReceiveD1(rBuffer,req.data,xjoint)){
	    pthread_mutex_lock( &brzdy_mutex);
	    kv01_fun->odbrzdene[xjoint] = true;
            pthread_mutex_unlock(&brzdy_mutex);
	    serv_ok=true;
	  }
	  else {
	    pthread_mutex_lock( &brzdy_mutex);
	    kv01_fun->odbrzdene[xjoint] = false;
            pthread_mutex_unlock(&brzdy_mutex);
	    delete [] rBuffer;
	    return false;
	  }
	  break;
	}
	
	default:
	{
	  res.error=comm_err_command;
	  ROS_ERROR("Chybny prikaz");
	  delete [] rBuffer;
	  return false; 
	}
      }
      delete [] rBuffer;
    }
    if(serv_ok)
      return true;
    else 
      return false;
  }
  
  
bool wifi_restart(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
	ROS_INFO("Restartujem wifi");
	pthread_mutex_lock(&wifi_mutex);
	wifi_fail = false;
	control=ros::Duration(0,500000000); 
	pthread_mutex_unlock( &wifi_mutex);
	return true;
}
  

int main(int argc, char **argv)
{  
  
  ros::init(argc, argv, "kv01_driver_servis");
  ros::NodeHandle n;
  
  std::string port = "/dev/ARM";
  long int baud = 115200;
  
  kv01_fun = new common_functions();
  
  kv01_fun->init_driver(); 
  
  pthread_t control_thread,wifi_thread,joint_states_thread;
  pthread_attr_t control_param,wifi_param,joint_states_param;
  ros::MultiThreadedSpinner spinner(4);
 
  reset_voltage=n.serviceClient<std_srvs::SetBool>("/mrvk/set_arm_voltage");
  
  my_serial= new serial::Serial(port,baud, serial::Timeout::simpleTimeout(2));
  sleep(2);
  if(my_serial->isOpen())
    ROS_INFO("Otovril sa serial port");
  else
  {
    ROS_INFO("Neuspesne otvorenie portu");
  return 0;
  }
  
  // inicializacia vlakna na kontrolu pozicie
 
  if (pthread_attr_init(&control_param)) 
  {		
    ROS_ERROR("Unable to create control_thread");
    ros::shutdown();
  }
  else
  {
    pthread_attr_setdetachstate(&control_param, PTHREAD_CREATE_DETACHED);
    
    if(pthread_create(&control_thread,&control_param ,*position_control_thread, (void*) &n))  
      ROS_ERROR("Unable to create position_control_thread"); 
  }
   // inicializacia vlakna na kontrolu funkcnej wifi
  if (pthread_attr_init(&wifi_param)) 
  {		
    ROS_ERROR("Unable to create wifi_thread");
    ros::shutdown();
  }
  else
  {
    pthread_attr_setdetachstate(&wifi_param, PTHREAD_CREATE_DETACHED);
    
    if(pthread_create(&wifi_thread,&wifi_param ,*live_wifi_thread,(void*) &n))  
      ROS_ERROR("Unable to create wifi_thread"); 
  }
 
 // inicializacia vlakna na citanie pozicie
  if (pthread_attr_init(&joint_states_param)) 
  {		
    ROS_ERROR("Unable to create Joint_states_thread");
    ros::shutdown();
  }
  else
  {
    pthread_attr_setdetachstate(&joint_states_param, PTHREAD_CREATE_DETACHED);
    
    if(pthread_create(&joint_states_thread,&joint_states_param ,*joint_states_fun_thread,(void*) &n))  
      ROS_ERROR("Unable to create wifi_thread"); 
  }
  
  ros::ServiceServer service = n.advertiseService("/kv01/driver_servis", WriteAndRead);
  ros::Subscriber wifi_topic = n.subscribe("/kv01/wifi_live", 1, wifi_live);
  ros::Subscriber joint_speed = n.subscribe("/kv01/joint_speed", 12, joint_speed_callback);
  ros::ServiceServer wifi_service = n.advertiseService("/kv01/wifi_restart", wifi_restart);
  
  spinner.spin();
  return 0;
}
