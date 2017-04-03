#include <kv01_driver/common_functions.h>

common_functions::common_functions() {
  //int x;
  //inicializacia premennych

  //prevodovka vypocet kohut 3682
  machine_data.maxvel_j[0]=((int) 9700); 		///ot/min
  machine_data.offset_j[0]=((short) 50636);
  machine_data.Ulimit_j[0]=((double)  2.094395102393195);
  machine_data.Dlimit_j[0]=((double) -2.094395102393195);
  machine_data.velocityConst_j[0]=((double) 35159.80374331211f);	///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
  machine_data.positionConst_j[0]=((double) 1635778.505157406f);	///prepoctova konstanta z uhla na impulzy pre faulhaber
  machine_data.commAddress_j[0]=0xE1;
  machine_data.Acclimit_j[0]=((double) 0.5);			///limit akceleracie v rad/s
  
  //prevodovka vypocet kohut 2547
  machine_data.maxvel_j[1]=((int) 4700); 		///ot/min
  machine_data.offset_j[1]=((short) 34693);
  machine_data.Ulimit_j[1]=((double)  1.570796326794897);
  machine_data.Dlimit_j[1]=((double) -1.570796326794897);
  machine_data.velocityConst_j[1]=((double) 24322.04065464460f);	///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
  machine_data.positionConst_j[1]=((double) 1210020.656644189f);	///prepoctova konstanta z uhla na impulzy pre faulhaber
  machine_data.commAddress_j[1]=0xE2;
  machine_data.Acclimit_j[1]=((double) 0.5);			///limit akceleracie v rad/s
  
  //prevodovka vypocet kohut 8178
  machine_data.maxvel_j[2]=((int) 20000); 		///ot/min
  machine_data.offset_j[2]=((short) 16741);
  machine_data.Ulimit_j[2]=((double)  2.303834612632515);
  machine_data.Dlimit_j[2]=((double) -2.303834612632515);
  machine_data.velocityConst_j[2]=((double) 78094.17541246234f);	///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
  machine_data.positionConst_j[2]=((double) 3474939.385799177f);	///prepoctova konstanta z uhla na impulzy pre faulhaber
  machine_data.commAddress_j[2]=0xE3;
  machine_data.Acclimit_j[2]=((double) 0.5);	///limit akceleracie v rad/s	
  
  //prevodovka vypocet kohut 6083.64
  machine_data.maxvel_j[3]=((int) 20000); 		///ot/min
  machine_data.offset_j[3]=((short) 46868);
  //machine_data.offset_j[3]=((short) 14100);
  machine_data.Ulimit_j[3]=((double)  2*M_PI); ///v skutocnosti bez limitu
  machine_data.Dlimit_j[3]=((double) -2*M_PI); ///v skutocnosti bez limitu
  machine_data.velocityConst_j[3]=((double) 58094.46928133084f);	///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
  machine_data.positionConst_j[3]=((double) -2536794.978523511f);	///prepoctova konstanta z uhla na impulzy pre faulhaber
  machine_data.commAddress_j[3]=0xE4;
  machine_data.Acclimit_j[3]=((double) 0.5);			///limit akceleracie v rad/s
  
  //prevodovka vypocet kohut 6083.64
  machine_data.maxvel_j[4]=((int) 20000); 		///ot/min
  machine_data.offset_j[4]=((short) 45593);//45957); posun o 182 predstavuje priblizne 2 stupen povodna hodnota bola 45957 pre tento klb som ju zmenil aby sa dostal do spravnej polohy
  machine_data.Ulimit_j[4]=((double) 2.356194490192345);
  machine_data.Dlimit_j[4]=((double) -2.356194490192345);
  machine_data.velocityConst_j[4]=((double) 52051.55225633789f);	///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
  machine_data.positionConst_j[4]=((double) 2214223.815362408f);	///prepoctova konstanta z uhla na impulzy pre faulhaber
  machine_data.commAddress_j[4]=0xE5;
  machine_data.Acclimit_j[4]=((double) 0.5);			///limit akceleracie v rad/s
  
  //prevodovka vypocet kohut 423
  machine_data.maxvel_j[5]=((int) 6000); 		///ot/min
  machine_data.offset_j[5]=((short) 1496); //3544);
  machine_data.Ulimit_j[5]=((double)  2*M_PI);  ///v skutocnosti bez limitu
  machine_data.Dlimit_j[5]=((double) -2*M_PI); ///v skutocnosti bez limitu*/
  machine_data.velocityConst_j[5]=((double) 4034.826486978523f);    ///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
  machine_data.positionConst_j[5]=((double) 127323.9565974565f);    ///prepoctova konstanta z uhla na impulzy pre faulhaber
  machine_data.commAddress_j[5]=0xE6;
  machine_data.Acclimit_j[5]=((double) 1.0);			///limit akceleracie v rad/s
  
  // konstanty na vypocet
  machine_data.voltage24Vconst=((double) 0.02740234375f);
  machine_data.voltage12Vconst=((double) 0.01310546875f);
  machine_data.voltage5Vconst=((double) 0.006552734375f);
  machine_data.angle_const=((double)0.00009587379924285257f);	///sluzi na prepocet do radianov
  machine_data.angle_const_phi=((double)0.001533980787886f);	///sluzi na prepocet do radianov
  machine_data.angle_const_dec=((double)57.2957795130f);       /// prepocet do stupnov
	
  /*for(x=0;x<6;x++) 
  {
      machine_data.maxvel_rad_j[x]=((double) machine_data.maxvel_j[x])/machine_data.velocityConst_j[x];
      fprintf(stdout, "mach data vel %15.10lf klbu cislo :%d \n", machine_data.maxvel_rad_j[x],x);
  }*/
}


common_functions::~common_functions(){
  // TODO Auto-generated destructor stub
}



bool common_functions::init_driver()
{
  for (int joint=0;joint<6;joint++)
  {
    odbrzdene[joint]=false;
    std::string joint_string = common_functions::to_string(joint);
    ros::param::set("/kv01/odbrzdeneklb"+joint_string,false); 
    ros::param::set("/kv01/volt5V_jklb"+joint_string,0);
    ros::param::set("/kv01/volt12V_jklb"+joint_string,0);
    ros::param::set("/kv01/volt24V_jklb"+joint_string,0);
    ros::param::set("/kv01/motorklb"+joint_string,false); 
    ros::param::set("/kv01/speedklb"+joint_string,0);
  }
  ros::param::set("/kv01/odbrzdeneklb5",true);
  odbrzdene[5]=true;
  return true;
}
uint8_t common_functions::Calculate_CRC(uint8_t *buffer, unsigned int num) 
{

  uint8_t CRC = 0x00;
  uint8_t A = 0, B=0, C=0, i=0 ,j = 0, k = 0;
  for(i=0;i<num;i++) 
  {
    A = buffer[i];
    for(k = 0; k < 8; k++) 
    {
      C = A^B;
      
      if (C & 0x01) 
      {
	C = ((B^0x18)/2)|0x80;
      }
      else
      {
	C = B/2;
      }
      
      B = C;
      if (A & 0x01) 
      {
	A = (A/2)|0x80;
      }
      else
      {
	A = A/2;
      }
     }
  }
  CRC = B;

  if(CRC==0xf0) 
  {
    CRC=0xf1;
  }
  return CRC;
}
unsigned short common_functions::Calculate_HL(uint8_t H, uint8_t L) 
{ 
  unsigned short a;
  a=H << 8;
  a=L | a;

  return a;
}

std::string common_functions::to_string(int i)
{
    std::stringstream ss;
    ss << i;
    return ss.str();
}
bool common_functions::check_response(uint8_t* buffer){
  responsedata=((DATA_OK_COMMAND *)buffer);
  if (responsedata->data1 == OK_data1 && responsedata->data2 == OK_data2)
    return true;
  else
    return false;     
}



uint8_t* common_functions::ArmCreateD0(uint8_t data, int joint, double value)
{ 
  unsigned int message_size;
  char string[25];
  int speed;
  
  switch(data) 
  {
    case comm_motory_en:
      sprintf(string, "EN");
      break;
    case comm_motory_dis:
      sprintf(string, "DI");
      break;
    case comm_status:
      sprintf(string, "GST");
      break;
    case comm_motory_speed:
    {
      int speed;
      value=value*machine_data.velocityConst_j[joint];
     // ROS_INFO("rychlost double %lf", value);
      speed=((int) value);
    //  ROS_INFO("rychlost %d", speed);
      if (speed >= 0){
      if(abs(speed)>(machine_data.maxvel_j[joint]/2))
	speed = (machine_data.maxvel_j[joint]/2);
      sprintf(string, "V%d", speed);
      }
      else{
	if(abs(speed)>(machine_data.maxvel_j[joint]/2))
	speed = (machine_data.maxvel_j[joint]/2);
      sprintf(string, "V-%d", speed);
      }
      break;
    }
    default:
    {
      uint8_t *err_message = new uint8_t[1];
      ROS_ERROR("Error: common_functions::ArmCreateD0: Neznamy variant dat prikazu D0 \n");
      err_message[0]=comm_err_data;
      return err_message;
    }
  }
  message_size=5L+strlen(string);
  requestD0 = new uint8_t [message_size];
  
  *(requestD0+0)=machine_data.commAddress_j[joint];			///naplnenie adresy
  *(requestD0+1)=message_size;			///naplnenie dlzky
  *(requestD0+2)=0xd0;			///prikaz
    for(int i=1;i<=strlen(string);i++) 
      *(requestD0+i+2)=string[i-1];
  *(requestD0+message_size-2)=0xf0;
  *(requestD0+message_size-1)=0xf0;
  *(requestD0+message_size-2)=Calculate_CRC(requestD0,message_size-1);
  
  return requestD0;
}

bool common_functions::ArmReceiveD0(uint8_t *buffer,int joint,uint8_t data)
{
  switch (data)
  {
    //TODO UPRAVA STATUSOV
   /* case comm_status:
    {
      for (int i = 2; i<9;i++)
	cout << buffer[i] << " ";
      cout << "" << endl;
      break;
    }*/
  }
  return true;
      
}

bool common_functions::ArmReceiveD1(uint8_t *buffer,uint8_t data, int joint)
{
  
  responseD1.data = buffer[3];
  std::string joint_string = common_functions::to_string(joint);
  
  switch(data)
  {
    case comm_brzdy_run:
    {
      if (responseD1.data & 0x08 && buffer[0] != 0xE6 )
      {
	//ros::param::set("/kv01/odbrzdeneklb"+joint_string,true);
	return true;
      }
      else if ((buffer[0]== 0xE6 && responseD1.data & 0x01))
      {
	//ros::param::set("/kv01/odbrzdeneklb"+joint_string,true);
	return true;
      }
      else
      {
	//ros::param::set("/kv01/odbrzdeneklb"+joint_string,false);
	return false;
      }
    }
    case comm_brzdy_stop:
    {
      if (!(responseD1.data | 0x00))
      {
	//ros::param::set("/kv01/odbrzdeneklb"+joint_string,false);
	return true;
      }
      else if (responseD1.data & 0x08)
      {
	//ros::param::set("/kv01/odbrzdeneklb"+joint_string,true);
	return false;
      }
      else
	return false;
    }
    
    return false;
  }

}

uint8_t* common_functions::ArmCreateD1(uint8_t *message,int joint, uint8_t data)
{
     
  requestD1.address=machine_data.commAddress_j[joint];
  requestD1.length=sizeof(requestD1);
  requestD1.command=0xD1;
  requestD1.crc=0xf0;
  requestD1.mode = data;
  requestD1.end=0xf0;
  message=((uint8_t *) &requestD1);     
  requestD1.crc=common_functions::Calculate_CRC(message, sizeof(requestD1)-1); 
  return message;
} 

uint8_t* common_functions::ArmCreateD3(uint8_t *message,int joint)
{
     
  requestD3.address=machine_data.commAddress_j[joint];
  requestD3.length=sizeof(requestD3);
  requestD3.command=0xD3;
  requestD3.crc=0xf0;
  requestD3.end=0xf0;
  message=((uint8_t *) &requestD3);     
  requestD3.crc=common_functions::Calculate_CRC(message, sizeof(requestD3)-1); 
  return message;
} 

double common_functions::ArmReceiveD3(uint8_t *buffer,int joint) 
{
  //funkcia spracuje odpoved na spravu D3 vypocita pozicie klbu a zapise do parametra ci je odbrzdeny alebo nahodou nie je mimo pracovnu polohu
  // definovanie smernika na strukturu pre sravu D3
  HAR_ARM_RES_D3 *p;
  //pomocne premenne
  unsigned short angle_compute;
  bool brzda[5];
  double x; 
  //naplnenie vektora p hodnotami z bufera kontrola jednotlivych bytov
  p=((HAR_ARM_RES_D3 *)buffer);
  if(p->length!=8) return -10;		///zla dlzka spravy spracovat hodnotu v maine a osetrit chybu
  //treba sparsovat prijate data
  if(p->data3&0x20) 
  p->data1=p->data1|0x80;
  if(p->data3&0x10) 
  p->data2=p->data2|0x80;
  
  
  //zapis hodnoty brzy do parametroveho servera 
  //true - odbrzdene
  //false - zabrzdene
  
  //Na raspberry spomaluje komunikaciu muselo byt zakomentovane dorobit globalne premenne

  if(joint<5 && joint>=0) 
  {
    pthread_mutex_lock( &brzdy_mutex);
    
    if(p->data3&0x08 && joint != 5)
      odbrzdene[joint]=true;
    else if (joint != 5)
      odbrzdene[joint]=false;
    
    pthread_mutex_unlock( &brzdy_mutex); 
    
    
    ///parsujem data pre klby 0-4
    angle_compute=common_functions::Calculate_HL(p->data1, p->data2);
    angle_compute-=machine_data.offset_j[joint];
    //vypocet uhla pre jednotlivy klb
    switch(joint) 
    {
      case 0:
      {
	rob_arm_position.alpha=-angle_compute*machine_data.angle_const; 
	x=rob_arm_position.alpha;//*machine_data.angle_const_dec;
	//printf("Uhol prveho klbu %15.10lf \n",x);		
	break;
      }
      case 1:
      {
	rob_arm_position.beta=angle_compute*machine_data.angle_const; 
	x=rob_arm_position.beta;//*machine_data.angle_const_dec;
	//printf("Uhol druheho klbu %15.10lf\n",x);		
	break;
      }
      case 2:
      {
	rob_arm_position.gamma=-angle_compute*machine_data.angle_const; 
	x=rob_arm_position.gamma;//*machine_data.angle_const_dec;
	//printf("Uhol tretieho klbu %15.10lf\n",x);		
	break;
      }
      case 3:
      {
	rob_arm_position.delta=-angle_compute*machine_data.angle_const; 
	x=rob_arm_position.delta;//*machine_data.angle_const_dec;
	//printf("Uhol stvrteho klbu%15.10lf\n",x);		
	break;
      }
      case 4:
      {
	rob_arm_position.theta=angle_compute*machine_data.angle_const; 
	x=rob_arm_position.theta;//*machine_data.angle_const_dec;
	//printf("Uhol piateho klbu%15.10lf\n",x);		
	break;
      }
    }
  }
  else
  { 
    //parsuejm data pre 5klb - ine to nemoze byt lebo joint bol skontrolovany este pred volanim funkcie
    angle_compute = common_functions::Calculate_HL(p->data1, p->data2);
    //rob_arm_position.phi=-angle_compute*machine_data.angle_const_phi;// uprava miroslav kohút zla konstatna
    rob_arm_position.phi=angle_compute*machine_data.angle_const;
    x=rob_arm_position.phi;//*machine_data.angle_const_dec;	
    
  }
  
  // uprava hodnot pre spravne fungovanie so simulaciou
  x= fabs(x);
  if (x > PHI)
    x = x-2*PHI;
  if (joint == 5)
    x=-x;
  if ((joint == 0) && (x <= 0))
    x = fabs(x);
  else if (joint == 0)
    x= x-2*x;
  return x;		
}
uint8_t* common_functions::ArmCreateD4(uint8_t *message,int joint)
{
  
  requestD4.address=machine_data.commAddress_j[joint];
  requestD4.length=sizeof(requestD4);
  requestD4.command=0xD4;
  requestD4.crc=0xf0;
  requestD4.end=0xf0;
  message=((uint8_t *) &requestD4);     
  requestD4.crc=common_functions::Calculate_CRC(message, sizeof(requestD4)-1); 
  return message;
} 


void common_functions::ArmReceiveD4(uint8_t *buffer) 
{
  ///ide o odpoved na prikaz d4
  HAR_ARM_RES_D4 *p;
  unsigned short voltage_compute;

  p=((HAR_ARM_RES_D4 *)buffer);	
 
  voltage_compute=(common_functions::Calculate_HL(p->volt24H, p->volt24L));
  rob_arm_voltages.volt24V_j=((double) voltage_compute*machine_data.voltage24Vconst);
	
  voltage_compute=( common_functions::Calculate_HL(p->volt12H, p->volt12L));
  rob_arm_voltages.volt12V_j=((double) voltage_compute*machine_data.voltage12Vconst);
  
  voltage_compute=(common_functions::Calculate_HL(p->volt5H, p->volt5L));
  rob_arm_voltages.volt5V_j=((double) voltage_compute*machine_data.voltage5Vconst);
			
  return;
}

uint8_t* common_functions::ArmCreateD5(uint8_t *message,int joint) {

  requestD5.address=0xE6;
  requestD5.length=sizeof(requestD5);
  requestD5.command=0xD1;
  //request.data=0x00;

  switch(joint)
  {
    case 1:
    {
      fprintf(stdout, "otvaram celust\n");
      //request.data=arm_set.SetGripVelocity;
      //request.data=0x20 | request.data;
      requestD5.data=0xC1;
      break;
    }
    case 2:
    {
      requestD5.data=0xC0;
      fprintf(stdout, "zatvaram celust\n");
      break;
    }
    case 3:
    {
      fprintf(stdout, "Stop celust\n");
      requestD5.command=0xD5;
      requestD5.data=0x7f;
      break;
    }
  }
  
  requestD5.crc=0xf0;
  requestD5.end=0xf0;
  message=((uint8_t *) &requestD5); 
  requestD5.crc=common_functions::Calculate_CRC(message, sizeof(requestD5)-1); 
  
  return message;
}

