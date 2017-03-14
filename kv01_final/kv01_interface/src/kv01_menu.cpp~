#include "kv01_driver/driver.h"
#include "kv01_driver/common_functions.h"
#include "kv01_driver/joint_speed.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Int8.h"
#include "std_srvs/Empty.h"

class kv01_menu
{
private: 
  
public:
  
  ros::NodeHandle n;
  ros::ServiceClient driver_client, wifi_client;
  kv01_driver::driver driver_client_data; 
  ros::Subscriber sub;
  std_srvs::Empty empty_request;
  ros::Publisher speed_pub;
  double position[6];
  bool no_positions;
  
  kv01_menu()
  {
    driver_client = n.serviceClient<kv01_driver::driver>("/kv01/driver_servis");
    wifi_client = n.serviceClient<std_srvs::Empty>("/kv01/wifi_restart"); 
    speed_pub = n.advertise<kv01_driver::joint_speed>("/kv01/joint_speed", 12);
    sleep(1);
    print_menu();
  }
 
 void print_menu(void)
  {
    cout << "\x1B[2J\x1B[H";
    cout << "||*********************************************************************||" << endl;
    cout << "||*********************** RIADENIE RAMENA KV01 ************************||" << endl;
    cout << "||*********************************************************************||" << endl;
    cout << "||ZVOLTE :                                                             ||" << endl;
    cout << "||1 :  Pre aktivovanie motorov a odbrzdenie                            ||" << endl;
    cout << "||2 :  Pre deaktivovanie motorov a zabrzdenie                          ||" << endl;
    cout << "||3 :  Pre vycitanie napati                                            ||" << endl;
    cout << "||4 :  Pre odbrzdenie konkretneho klbu                                 ||" << endl;
    cout << "||5 :  Pre zabrzdenie konkretneho klbu                                 ||" << endl;
    cout << "||6 :  Pre aktivaciu  konkretneho motora                               ||" << endl;
    cout << "||7 :  Pre deaktivaciu konkretneho motora                              ||" << endl;
    cout << "||8 :  Pre vycitanie statusu                                           ||" << endl;
    cout << "||9 :  Pre vycitanie polohy klbov                                      ||" << endl;
    cout << "||10:  Pre riadenie grippera /nefunkcne                                ||" << endl;
    cout << "||11:  Pre okamzite zastavenie vsetkych motorov                        ||" << endl;
    cout << "||12:  Pre riadenie robota pomocou simulácie (potrebný bod 1)//nejde   ||" << endl;
    cout << "||13:  Pre umiestnenie klbu na lubovolnu poziciu                       ||" << endl;
    cout << "||14:  Pre prechod do home pozície                                     ||" << endl;
    cout << "||15:  Reset komunikacie                                               ||" << endl;
    cout << "||-1:  Vo vyvoji                                                       ||" << endl;
    cout << "||-2:  Pre ukoncenie menu (deaktivuju sa motory a zabrzdia klby a vypne||" << endl;
    cout << "||     sa komunikaca) => nutne restartnutie napajania ramena           ||" << endl;
    cout << "||-3:  Pre znovuzobrazenie menu a vycistenie plochy                    ||" << endl;
    cout << "||/////////////////////////////////////////////////////////////////////||" << endl;  
    cout << "" << endl;
  }
  
  void flush_stream(std::istream& stream)
  {
    stream.clear();
    stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  
  bool request_for_driver(int *joint,uint8_t command,uint8_t data)
  {
    kv01_driver::driver driver_client_data;
    for (int i =0;i<6;i++){
    driver_client_data.request.joint[i]=joint[i]; 
    } 
    driver_client_data.request.command=command;
    driver_client_data.request.data=data;
    //driver_client.waitForExistence();
    if (driver_client.call(driver_client_data))
	{
	  return true;
	}
	else
	{
	 // ROS_INFO("Niekde vznikla chyba");
	  return false;
	} 
  }
  
  bool speed_pos_pub(int joint,double speed,double pos)
  {
    kv01_driver::joint_speed joint_speed;
    joint_speed.command = comm_position;
    joint_speed.joint = joint; 
    joint_speed.speed = speed;
    joint_speed.position = pos;
    speed_pub.publish(joint_speed);
  }
  
  
  string to_string(int i)
  {
    std::stringstream ss;
    ss << i;
    return ss.str();
  }
  
void Callback(const sensor_msgs::JointState::ConstPtr& joint)
  {
    for (int i=0;i<6;i++)
    {
      position[i]=joint->position[i];
    }
    no_positions= false;  
  }
  
};

void *live_wifi_thread(void *node)
{
  ros::Rate loop_rate(8);
  ros::NodeHandle *n=(ros::NodeHandle *) node;
  ros::Publisher wifi = n->advertise<std_msgs::Int8>("/kv01/wifi_live",1);
  std_msgs::Int8 fake;
  fake.data= 1;
  while(ros::ok())
  {
    wifi.publish(fake);
    loop_rate.sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "kv01_menu");
  kv01_menu* menu_fun = new kv01_menu();
  kv01_driver::joint_speed joint_speed;
  
  pthread_t wifi_thread;
  pthread_attr_t wifi_param;
    if (pthread_attr_init(&wifi_param)) 
  {		
    ROS_ERROR("Unable to create wifi_thread");
    ros::shutdown();
    return 0;
  }
  else
  {
    pthread_attr_setdetachstate(&wifi_param, PTHREAD_CREATE_DETACHED);
    
    if(pthread_create(&wifi_thread,&wifi_param ,*live_wifi_thread,(void*) &menu_fun->n))  
      ROS_ERROR("Unable to create wifi_thread"); 
  }
  
  int prikaz;
  int joint;
  bool help;
  double speed;
  std::string joint_string;

  while(ros::ok())
  {
    help=true;
    cout << "||PRIKAZ||:" << flush;
    cin >> prikaz;
    switch(prikaz)
    {
      case -1:
      {
	return 1;
      }
      case -2:
      {
	return 1;
      }
      case -3:
      {
	menu_fun->print_menu();
	break;
      }
      case 1:
      {
	int joint_message[6]={1,1,1,1,1,1};
	if(menu_fun->request_for_driver(joint_message,comm_message,comm_motory_en))
	{
	  joint_message[5]=0;
	  if(!menu_fun->request_for_driver(joint_message,comm_brzdy,comm_brzdy_run))
	  help=false;  
	}
	else {
	  help=false;
	} 
	if(help)
	  ROS_INFO("MOTORY SA USPESNE AKTIVOVALI A ODBRZDILI\n");
	else
	  ROS_ERROR("CHYBA PRI AKTIVACII A ODBRZDENI MOTOROV\n");      
	break;
      }
      case 2:
      {   
	  int joint_message[6]={1,1,1,1,1,0};
	  if(menu_fun->request_for_driver(joint_message,comm_brzdy,comm_brzdy_stop))
	  {
	    joint_message[5]=1;
	    if(menu_fun->request_for_driver(joint_message,comm_message,comm_motory_dis))
		help=true;
	    else
		help=false;
	  } 
	  else
	    help = false;
	if(help)
	  ROS_INFO("MOTORY SA USPESNE DEAKTIVOVALI A ZABRZDILI\n");
	else
	  ROS_ERROR("CHYBA PRI DEAKTIVACII A ZABRZDENI MOTOROV\n");      
	break;
      }
      
      case 3:
      {
	int joint_message[6]={1,1,1,1,1,1}; 
	if(!menu_fun->request_for_driver(joint_message,comm_voltages,0))
	{
	  help=false;
	}
	
	if(help)
	{
	  double voltages[3];
	  ROS_INFO("NAPATIA SA VYCITALI NA PARAMETROVY SERVER\n");
	  for(int i=0;i<6;i++)
	  {
	    joint_string = menu_fun->to_string(i);
	    ros::param::get("/kv01/volt5V_jklb"+joint_string,voltages[0]);
	    ros::param::get("/kv01/volt12V_jklb"+joint_string,voltages[1]);
	    ros::param::get("/kv01/volt24V_jklb"+joint_string,voltages[2]);
	    cout << "Napatie na 5V casti klbu"  << i << " je " << voltages[0] <<  endl;
	    cout << "Napatie na 12V casti klbu" << i << " je " << voltages[1] <<  endl;
	    cout << "Napatie na 24V casti klbu" << i << " je " << voltages[2] <<  endl;
	  }
	}  
	else
	  ROS_ERROR("CHYBA PRI CITANI NAPATI\n");      
	break;
      }
      case 4:
      {
	 int joint_message[6]={0,0,0,0,0,0}; 
	 cout << "||Ktory z klbov si prajete odbrzdit ? (0-5)||" << endl;
	 cout << "||Zadajte cislo klbu||:" << flush;
	 cin >> joint;
	 if (joint <0 || joint>5)
	 {
	   ROS_ERROR("Chybne cislo klbu");
	   break;
	 }
	 joint_message[joint]=1;
	 if(menu_fun->request_for_driver(joint_message,comm_brzdy,comm_brzdy_run))
	   ROS_INFO("Klb cislo %d bol uspesne odbrzdeny", joint);
	 else
	   ROS_ERROR("Klb cislo %d sa nepodarilo odbrzdit", joint);

	break;
      }
      case 5:
      {
	 int joint_message[6]={0,0,0,0,0,0};
	 cout << "||Ktory z klbov si prajete zabrzdit ? (0-5)||" << endl;
	 cout << "||Zadajte cislo klbu||:" << flush;
	 cin >> joint;
	 if (joint <0  || joint>5)
	 {
	   ROS_ERROR("Chybne cislo klbu");
	   break;
	 }
	 joint_message[joint]=1;
	 if(menu_fun->request_for_driver(joint_message,comm_brzdy,comm_brzdy_stop))
	   ROS_INFO("Klb cislo %d bol uspesne zabrzdeny", joint);
	 else
	   ROS_ERROR("Klb cislo %d sa nepodarilo zabrzdit", joint);

	break;
      }
      case 6:
      {  
	 int joint_message[6]={0,0,0,0,0,0};
	 cout << "||Ktory z klbov si prajete aktivovat ? (0-5)||" << endl;
	 cout << "||Zadajte cislo klbu||:" << flush;
	 cin >> joint;
	 if (joint <0 || joint>5)
	 {
	   ROS_ERROR("Chybne cislo klbu");
	   break;
	 }
	 joint_message[joint]=1;
	 if(menu_fun->request_for_driver(joint_message,comm_message,comm_motory_en))
	   ROS_INFO("Motor cislo %d bol uspesne aktivovany", joint);
	 else
	   ROS_ERROR("Motor cislo %d sa nepodarilo aktivovat", joint);

	break;
      }      
      case 7:
      {  
	 int joint_message[6]={0,0,0,0,0,0};
	 cout << "||Ktory z klbov si prajete deaktivovat ? (0-5)||" << endl;
	 cout << "||Zadajte cislo klbu||:" << flush;
	 cin >> joint;
	 if (joint <0 || joint>5)
	 {
	   ROS_ERROR("Chybne cislo klbu");
	   break;
	 }
	 joint_message[joint]=1;
	 if(menu_fun->request_for_driver(joint_message,comm_message,comm_motory_dis))
	   ROS_INFO("Motor cislo %d bol uspesne deaktivovany", joint);
	 else
	   ROS_ERROR("Motor cislo %d sa nepodarilo deaktivovat", joint);

	break;
      }
      case 8:
      {
	 
        int joint_message[6]={1,1,1,1,1,1};
	if(!menu_fun->request_for_driver(joint_message,comm_status,comm_status))	
	  help=false;
	
	 if (help)
	   ROS_INFO("Status sa vypisal na konzole drivera");
	 else{
	   ROS_ERROR("Chyba pri citani status");
	   break;
	 }
	 
	double speed_write;
	bool brzdy_write;
	bool motory_write;
        for(int i=0;i<6;i++){
	      joint_string= menu_fun->to_string(i);
	      
	      ros::param::get("kv01/odbrzdeneklb"+joint_string,brzdy_write);
	      ros::param::get("kv01/motorklb"+joint_string,motory_write);
	      ros::param::get("kv01/speedklb"+joint_string,speed_write);
	      
	      cout << "|| Motor čislo " << i << " || " << endl;
	      if(motory_write)
		cout << "Stav motora: aktívny" << endl;
	      else
		cout << "Stav motora: neaktívny" << endl;
	      if(brzdy_write)
		cout << "Stav brzd: odblokované" << endl;
	      else
		cout << "Stav brzd: zablokované" << endl;
	      cout << "Rýchlost motora: " << speed_write << endl;
	}
      break;
      }
      case 9:
      {
	ROS_ERROR("Prikaz nefunkcny pre zistenie polohy odporucame napisat do konzoly \n:\"rostopic echo /kv01/joint_states\"");
	break;
      }
      case 10:
      {
	int grip_comm;
	cout << "||1: Otvorit Gripper ||" << endl;
	cout << "||2: Zatvorit Gripper||" << endl;
	cout << "||3: Zastavit Gripper||" << endl;
	cout << "||Zadajte cislo klbu||:" << flush;
	cin >> grip_comm;
	
	 if (grip_comm <1 || grip_comm > 3)
	 {
	   ROS_ERROR("Chybne cislo prikazu");
	   break;
	 }
	joint_speed.joint = grip_comm;
	joint_speed.command = comm_gripper_control;
        menu_fun->speed_pub.publish(joint_speed);
      }
      break;
      case 11:
      {
	for(int i=0;i<6;i++)
	{
	joint_speed.joint = i;
	joint_speed.command = comm_motory_speed;
	joint_speed.speed = 0;
	
        menu_fun->speed_pub.publish(joint_speed);
	}
	
	  ROS_INFO("MOTORY SA USPESNE ZASTAVILI\n");
	
      }
      break;
      case 12:
      {
	ROS_ERROR("!! Stale vo vyvoji !!\n");
	break;
	//system("gnome-terminal -x sh -c 'roslaunch kv01_sim moveit_gazebo_sim_final.launch'");
      }
      case 13:
      {
	double pos;

	cout << "||Ktory z klbov si prajete aktivovat ? (0-5)||" << endl;
	cout << "||Zadajte cislo klbu||:" << flush;
	cin >> joint;
	
	 if (joint <0 || joint>5)
	 {
	   ROS_ERROR("Chybne cislo klbu");
	   break;
	 }
     
	cout << "||Do akej pozicie v radianoch chcete ísť||" << endl;
	cout << "||Zadajte cislo pozicie||: " << flush;

	cin >> pos;
	
	cout << "||Akou rychlostou sa ma hybat max 0 - 0.1 rad/s||" << endl;
	cout << "||Zadajte rychlost||: "<< flush;
	cin >> speed;
	
	if (speed > 1.5 || speed < 0)
	{
	  ROS_ERROR("Nespravne data\n");
	  break;
	}
	menu_fun->no_positions = true;
	
	menu_fun->sub = menu_fun->n.subscribe("/kv01/joint_states", 1, &kv01_menu::Callback,menu_fun);
	
	while(ros::ok() && menu_fun->no_positions){
	  ros::spinOnce();
	}
	
	if (pos < menu_fun->position[joint])
	  speed =-speed;

	menu_fun->speed_pos_pub(joint,speed,pos);
	   ROS_INFO("Odoslana sprava na motor cislo %d spre pohyb rychlostou [rad/s] %lf \n",joint,speed);
	 
	break;
      }
      case 14:
      {
	ROS_INFO("Going Home v pripade zacyklenia je potrebne spustit driver");
	speed = 0.08;
	
	menu_fun->no_positions = true;
	menu_fun->sub = menu_fun->n.subscribe("/kv01/joint_states", 6, &kv01_menu::Callback,menu_fun);
	
	while(ros::ok() && menu_fun->no_positions){
	  ros::spinOnce();  
	} 
	
	for(int joint=0;joint<6;joint++)
	{
	  switch(joint){
	    case 0:{
	      
	      if (0 < menu_fun->position[joint])
		menu_fun->speed_pos_pub(0,-speed,0);
	      else
	        menu_fun->speed_pos_pub(0,speed,0);
	      break;
	    }
	    case 1:{
	      if (0.31< menu_fun->position[joint])
		menu_fun->speed_pos_pub(1,-speed,0.51);
	      else
	        menu_fun->speed_pos_pub(1,speed,0.51);
	      break;
	    }
	    case 2:{
	      if (1.9 < menu_fun->position[joint])
		menu_fun->speed_pos_pub(2,-speed,2.1);
	      else
	        menu_fun->speed_pos_pub(2,speed,2.1);
	      break;
	    }
	    case 3:{
	      if (0 < menu_fun->position[joint])
		menu_fun->speed_pos_pub(3,-speed,0);
	      else
	        menu_fun->speed_pos_pub(3,speed,0);
	      break;
	    }
	    case 4:{
	      if (1.9 < menu_fun->position[joint])
		menu_fun->speed_pos_pub(4,-speed,2.1);
	      else
	        menu_fun->speed_pos_pub(4,speed,2.1);
	      break;
	    }
	    case 5:{
	      if (0 < menu_fun->position[joint])
		menu_fun->speed_pos_pub(5,-speed,0);
	      else
	        menu_fun->speed_pos_pub(5,speed,0);
	      break;
	    }
	  }
	}
	break;
      }
      case 15:
      {
	int joint_message[6]={1,1,1,1,1,1};
	if (!menu_fun->wifi_client.call(menu_fun->empty_request))
	{
	  ROS_ERROR("Chyba pri resete komunikacie restartujte napajanie a driver");
	  break;
	}
	
	if(menu_fun->request_for_driver(joint_message,comm_message,comm_motory_en))
	{
	  joint_message[5]=0;
	  if(!menu_fun->request_for_driver(joint_message,comm_brzdy,comm_brzdy_run))
	  help=false;  
	}
	else {
	  help=false;
	} 
	if(help)
	  ROS_INFO("Komunikacie resetnuta motory aktivovane a odbrzdene\n");
	else
	  ROS_ERROR("Komunikacie resetnuta: Chyba pri odbrzdeni a aktiovani motorov po uspesnom resete komunikacie \n");      
	break;
	
      }
      break;
      default:
      {
	cout << "||Chybny prikaz||" << endl;
	menu_fun->flush_stream(cin);
      }
    }
    
  }
  ros::shutdown();
 return 0;
}
