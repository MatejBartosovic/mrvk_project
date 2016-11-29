#include "mrvk_driver/command.h"

/***Trieda MBCommand (pre Main board)  dedi metody z  triedy Command***/
//public
//constructor
MBCommand::MBCommand(uint8_t adresa) : Command(adresa),last_X_camera(0),last_Z_camera(0)
{
    rob_set_MB.MCBsSB_5V = true; //true
    rob_set_MB.MCBs_12V = true; //true
    rob_set_MB.videoTransmitter = false;
    rob_set_MB.wifi = true; //true
    rob_set_MB.laser = true;
    rob_set_MB.GPS = false;
    rob_set_MB.ARM_5V = false;
    rob_set_MB.ARM_12V = false;
    rob_set_MB.PC2 = true;
    rob_set_MB.kamera = false;
    
    rob_set_MB.centralStop = false;
    rob_set_MB.powerOff = false;
    rob_set_MB.fullBatAck = false;
    rob_set_MB.resetQbat = false;
   rob_set_MB.video1 = true;
   rob_set_MB.video2 = false;
   // defaultne nastavena otackova regulacia s rychlostami 0,0
   rob_set_MB.posRotCam = true; //false
   rob_set_MB.camAngleX = 0;
   rob_set_MB.camAngleZ = 0;
   
    //skopirovane s kodu robllrs422... treba odtestovat ci su tieto parametre ok
   rob_set_MB.pko=10; //15
   rob_set_MB.pkk=10;
   rob_set_MB.iko=40; //10
   rob_set_MB.ikk=80;

   rob_set_MB.commandID = REQUEST_COMMAND_FLAG;
   pthread_mutex_init(&MB_mutex, NULL);
}
//getery
int MBCommand::getControlCommand(uint8_t* command) //pomocou pola command naplni zakladny kontrol command
{
		command[0] = 0x03;
  	command[1] = 0x13;
  	command[2] = 0x80;
  	// pm1
		pthread_mutex_lock(&MB_mutex);	
  	command[4]=(rob_set_MB.MCBsSB_5V+2*rob_set_MB.MCBs_12V +4*rob_set_MB.videoTransmitter +8*rob_set_MB.wifi+ 
		16*rob_set_MB.laser+32*rob_set_MB.GPS+64*rob_set_MB.ARM_5V+128*rob_set_MB.ARM_12V);
		//pm2
		command[5]=rob_set_MB.PC2+2*rob_set_MB.kamera;
		//ctrl posledna nula 
		command[6]=rob_set_MB.centralStop+2*rob_set_MB.powerOff+4*rob_set_MB.fullBatAck+8*rob_set_MB.resetQbat+
		16*rob_set_MB.video1+32*rob_set_MB.video2+64*rob_set_MB.posRotCam;
	
	if (rob_set_MB.fullBatAck) rob_set_MB.fullBatAck = false;
	if (rob_set_MB.resetQbat) rob_set_MB.resetQbat = false;	
       //kamera 1H, 1L, 2H, 2L
        /*command[7] = 0x00;
 	command[8]=0x00;
	command[9]=0x00;
	command[10]=0x00;*/
	if (!rob_set_MB.posRotCam) //otackova regulacia
		kameraVelocity(&command[7],(double)rob_set_MB.camAngleZ,(double)rob_set_MB.camAngleX);
	else 
	 	kameraPosition(&command[7],(double)rob_set_MB.camAngleZ,(double)rob_set_MB.camAngleX);

	//kamera regulator
	command[12]=rob_set_MB.pko;
	command[13]=rob_set_MB.pkk;
	command[14]=rob_set_MB.iko;
	command[15]=rob_set_MB.ikk;
	rob_set_MB.commandID = REQUEST_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
	//rezerva
	command[16]=0; //0
	command[17]=0; //1
	command[18]=0; //3
	
	processParity(&command[3]);
	processParity(&command[11]);
	computeCRCWithParity(command,controlCommandLength);
	return 6; // pocet bytov, ktore sa budu ocakavat ako answer
}

int MBCommand::getPartialCommand(uint8_t* command) // sluzi na skrateni prikaz vyuzitelnost pri kamere
{
	command[0] = 0x03;
  	command[1] = 0x0B;
  	command[2] = 0x01;
  	//ctrl 
		pthread_mutex_lock(&MB_mutex);
  	command[4]=rob_set_MB.centralStop+2*rob_set_MB.powerOff+4*rob_set_MB.fullBatAck+8*rob_set_MB.resetQbat+
		16*rob_set_MB.video1+32*rob_set_MB.video2+64*rob_set_MB.posRotCam;
	
	if (rob_set_MB.fullBatAck) rob_set_MB.fullBatAck = false;
	if (rob_set_MB.resetQbat) rob_set_MB.resetQbat = false;
	
	if (!rob_set_MB.posRotCam) //otackova regulacia
		kameraVelocity(&command[5],(double)rob_set_MB.camAngleZ,(double)rob_set_MB.camAngleX);
	else 
	 	kameraPosition(&command[5],(double)rob_set_MB.camAngleZ,(double)rob_set_MB.camAngleX);
		rob_set_MB.commandID = REQUEST_COMMAND_FLAG;
		pthread_mutex_unlock(&MB_mutex);	
		command[9] = 0;
		command[10] = 1;
		processParity(&command[3]);
		computeCRCWithParity(command,partialCommandLength);
		return 25;
} 

int MBCommand::getUnitedCommand(uint8_t* command) //spojene citanie a zapis
{
command[0] = 0x03;
  	command[1] = 0x13;
  	command[2] = 0x02;
  	// pm1
		pthread_mutex_lock(&MB_mutex);	
  	command[4]=(rob_set_MB.MCBsSB_5V+2*rob_set_MB.MCBs_12V +4*rob_set_MB.videoTransmitter +8*rob_set_MB.wifi+ 
		16*rob_set_MB.laser+32*rob_set_MB.GPS+64*rob_set_MB.ARM_5V+128*rob_set_MB.ARM_12V);
		//pm2
	command[5]=rob_set_MB.PC2+2*rob_set_MB.kamera;
	//ctrl posledna nula 
	command[6]=rob_set_MB.centralStop+2*rob_set_MB.powerOff+4*rob_set_MB.fullBatAck+8*rob_set_MB.resetQbat+
		16*rob_set_MB.video1+32*rob_set_MB.video2+64*rob_set_MB.posRotCam;
		
	if (rob_set_MB.fullBatAck) rob_set_MB.fullBatAck = false;
	if (rob_set_MB.resetQbat) rob_set_MB.resetQbat = false;
       //kamera 1H, 1L, 2H, 2L
       /* command[7] = 0x00;
 	command[8]=0x00;
	command[9]=0x00;
	command[10]=0x00;*/
	if (!rob_set_MB.posRotCam) //otackova regulacia
		kameraVelocity(&command[7],(double)rob_set_MB.camAngleZ,(double)rob_set_MB.camAngleX);
	else 
	 	kameraPosition(&command[7],(double)rob_set_MB.camAngleZ,(double)rob_set_MB.camAngleX);
	//kamera regulator
	command[12]=rob_set_MB.pko;
	command[13]=rob_set_MB.pkk;
	command[14]=rob_set_MB.iko;
	command[15]=rob_set_MB.ikk;
	rob_set_MB.commandID = REQUEST_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
	//rezerva
	command[16]=0;
	command[17]=1;
	command[18]=3;
	processParity(&command[3]);
	processParity(&command[11]);
	computeCRCWithParity(command,unitedCommandLength);
	return 34; // pocet bytov, ktore sa budu ocakavat ako answer
}

bool MBCommand::getPosRotCam() //ak je true tak je polohova regulacia inak otackova
   {
	pthread_mutex_lock(&MB_mutex);	
	bool ret=rob_set_MB.posRotCam;
	pthread_mutex_unlock(&MB_mutex);
 	return ret;  
}

void MBCommand::getStructMain(SET_MAIN_BOARD* MB)
{
	pthread_mutex_lock(&MB_mutex);
	MB->MCBsSB_5V = rob_set_MB.MCBsSB_5V;
	MB->MCBs_12V = rob_set_MB.MCBs_12V;
	MB->videoTransmitter = rob_set_MB.videoTransmitter;
	MB->wifi = rob_set_MB.wifi;
	MB->laser = rob_set_MB.laser;
	MB->GPS = rob_set_MB.GPS;
	MB->ARM_5V = rob_set_MB.ARM_5V;
	MB->ARM_12V = rob_set_MB.ARM_12V;
		///pm2
	MB->PC2 = rob_set_MB.PC2;
	MB->kamera = rob_set_MB.kamera;
		///ctrl
	MB->centralStop = rob_set_MB.centralStop;
	MB->powerOff = rob_set_MB.powerOff;
	MB->fullBatAck = rob_set_MB.fullBatAck;
	MB->video1 = rob_set_MB.video1;
	MB->video2 = rob_set_MB.video2;
		///cam 
	MB->posRotCam = rob_set_MB.posRotCam;		//!< ak je true tak je polohova regulacia inak otackova
	MB->camAngleX = rob_set_MB.camAngleX;
	MB->camAngleZ = rob_set_MB.camAngleZ;
		///premenne regulacie - P a I regulator kamery
	MB->pko = rob_set_MB.pko;
	MB->pkk = rob_set_MB.pkk;
	MB->iko = rob_set_MB.iko;
	MB->ikk = rob_set_MB.ikk;
	pthread_mutex_unlock(&MB_mutex);	
}

int MBCommand::getCommandID(){
	pthread_mutex_lock(&MB_mutex);
	int ret = rob_set_MB.commandID;
	pthread_mutex_unlock(&MB_mutex);
	return ret;
}

int MBCommand::getCommandID(bool nuluj){
	pthread_mutex_lock(&MB_mutex);
	int ret = rob_set_MB.commandID;
	if (nuluj)
		rob_set_MB.commandID = REQUEST_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
	return ret;
}

//gettery
bool MBCommand::getCentralStop(){
	bool state;
	pthread_mutex_lock(&MB_mutex);
	state = rob_set_MB.centralStop;
	pthread_mutex_unlock(&MB_mutex);
	return state;
}
//settery
 void MBCommand::setParamatersMB_PM(bool MCBsSB_5V,bool MCBs_12V, bool videoTransmitter, bool wifi, bool laser,
    				  bool GPS, bool ARM_5V, bool ARM_12V, bool PC2, bool kamera)
    {
		pthread_mutex_lock(&MB_mutex);	
    rob_set_MB.MCBsSB_5V = MCBsSB_5V;
    rob_set_MB.MCBs_12V = MCBs_12V;
    rob_set_MB.videoTransmitter = videoTransmitter;
    rob_set_MB.wifi = wifi;
    rob_set_MB.laser = laser;
    rob_set_MB.GPS = GPS;
    rob_set_MB.ARM_5V = ARM_5V;
    rob_set_MB.ARM_12V = ARM_12V;
    rob_set_MB.PC2 = PC2;
    rob_set_MB.kamera = kamera;
    rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
		pthread_mutex_unlock(&MB_mutex);	
 }

 void MBCommand::setParamatersMB(SET_MAIN_BOARD* config)
    {
	pthread_mutex_lock(&MB_mutex);
    rob_set_MB.MCBsSB_5V = config->MCBsSB_5V;
    rob_set_MB.MCBs_12V = config->MCBs_12V;
    rob_set_MB.videoTransmitter = config->videoTransmitter;
    rob_set_MB.wifi = config->wifi;
    rob_set_MB.laser = config->laser;
    rob_set_MB.GPS = config->GPS;
    rob_set_MB.ARM_5V = config->ARM_5V;
    rob_set_MB.ARM_12V = config->ARM_12V;
    rob_set_MB.PC2 = config->PC2;
    rob_set_MB.kamera = config->kamera;
    rob_set_MB.video1 = config->video1;
    rob_set_MB.video2 = config->video2;
    rob_set_MB.pko = config->pko;
    rob_set_MB.pkk = config->pkk;
    rob_set_MB.iko = config->iko;
    rob_set_MB.ikk = config->ikk;
    rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
 }

 void MBCommand::setParametersMB_CTRL(bool centralStop, bool powerOff, bool fullBatAck, bool resetQbat, bool camera_source)
{		///ctrl
	 pthread_mutex_lock(&MB_mutex);	
   rob_set_MB.centralStop = centralStop;
   rob_set_MB.powerOff = powerOff;
   rob_set_MB.fullBatAck = fullBatAck;
   rob_set_MB.resetQbat = resetQbat;
   rob_set_MB.video1 = camera_source;
   rob_set_MB.video2 = !camera_source;
   rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	 pthread_mutex_unlock(&MB_mutex);	
  }
  void MBCommand::setKameraRegulator(uint8_t pko,uint8_t pkk,uint8_t iko, uint8_t ikk)
{	
	pthread_mutex_lock(&MB_mutex);	
	rob_set_MB.pko = pko;
	rob_set_MB.pkk = pkk;
	rob_set_MB.iko = iko;
	rob_set_MB.ikk = ikk;
	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
  } 
  
  void MBCommand::setKameraCommand(int Z, int X)
{
	pthread_mutex_lock(&MB_mutex);
	if((Z != last_Z_camera)||(X !=last_X_camera)){
		ROS_ERROR("hovno %d %d",X,Z);
		last_Z_camera = Z;
		last_X_camera = X;
		rob_set_MB.camAngleX = X;
		rob_set_MB.camAngleZ = Z;
		rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	}
	pthread_mutex_unlock(&MB_mutex);
  } 

   void MBCommand::setMCBsSB_5V(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.MCBsSB_5V= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	  
}
   void MBCommand::setVideoTransmitterSwitch(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.video1 = parameter;
 	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	  
}
   void MBCommand::setVideoGrabberSwitch(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.video2 = parameter;
 	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
 	pthread_mutex_unlock(&MB_mutex);

   }
   bool MBCommand::switchVideo(){
	 pthread_mutex_lock(&MB_mutex);
	 rob_set_MB.video1 = !rob_set_MB.video1;
	 rob_set_MB.video2 = !rob_set_MB.video2;
	 rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	 bool ret = rob_set_MB.video1;
	 pthread_mutex_unlock(&MB_mutex);
	 return ret;
   }
void MBCommand::setMCBs_12V(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.MCBs_12V= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	  
}
void MBCommand::setVideoTransmitter(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.videoTransmitter= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
}
 void MBCommand::setWifi(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.wifi= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
}
void MBCommand::setLaser(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.laser= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
}
void MBCommand::setGPS(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.GPS= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
}
void MBCommand::setKamera(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.kamera= parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
}
 void MBCommand::setCentralStop(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.centralStop= parameter;
 	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	
}
void MBCommand::setPowerOff(bool parameter)
   {
	pthread_mutex_lock(&MB_mutex);	
 	rob_set_MB.powerOff= parameter;
 	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);	 
}
void MBCommand::setPosRotCam(bool parameter) //ak je true tak je polohova regulacia inak otackova
   {
	pthread_mutex_lock(&MB_mutex);
	if(parameter!=rob_set_MB.posRotCam ){
		rob_set_MB.posRotCam = parameter;
		rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	}
	pthread_mutex_unlock(&MB_mutex);	
}

void MBCommand::setFullBatery(bool parameter) 
   {
	pthread_mutex_lock(&MB_mutex);
 	rob_set_MB.fullBatAck = parameter;
 	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
}
void MBCommand::setResetQBatery(bool parameter) 
   {
	pthread_mutex_lock(&MB_mutex);
 	rob_set_MB.resetQbat = parameter;
 	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
}

void MBCommand::resetBatery(){
	pthread_mutex_lock(&MB_mutex);
	rob_set_MB.resetQbat = true;
	rob_set_MB.fullBatAck = true;
	rob_set_MB.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
}
void MBCommand::setArm5V(bool parameter) 
   {
	pthread_mutex_lock(&MB_mutex);
 	rob_set_MB.ARM_5V = parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
}
void MBCommand::setArm12V(bool parameter) 
   {
	pthread_mutex_lock(&MB_mutex);
 	rob_set_MB.ARM_12V = parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
}
void MBCommand::setArmPower(bool parameter){
	pthread_mutex_lock(&MB_mutex);
	rob_set_MB.ARM_12V = parameter;
	rob_set_MB.ARM_5V = parameter;
	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
	pthread_mutex_unlock(&MB_mutex);
}
void MBCommand::setPC2(bool parameter) 
   {
	pthread_mutex_lock(&MB_mutex);
 	rob_set_MB.PC2 = parameter;
 	rob_set_MB.commandID |= UNITED_COMMAND_FLAG;
 	pthread_mutex_unlock(&MB_mutex);
}
//private triedy MBCommand
void MBCommand::kameraVelocity(uint8_t* command,double z, double x)
{
		int y;		
		if(x>255.) x=255.; 
		if(x<-255.) x=-255.;
		if(x>0.){
			y = (int)(x + (MBOFS1 * ( 1 - (x/255.) )));
		}else{ 
			if(x<0.){
				y = (int) (x -(MBOFS1 * ( 1 + (x/255.))));
			}else {
				y=0;
			} 
		}
		
		command[0]=(y>>8) & 0xff;
		command[1]=y & 0xff;
	
		if(z>255.) z=255.; 
		if(z<-255.) z=-255.;
		if(z>0.){
			y =(int) (z + ( MBOFS2 * ( 1 - (z/255.) )));
		}else{
			if(x<0.){
				y = (int) z - (( MBOFS2 * ( 1 + (z/255.) )));
			}else{
				y = 0.0;
			}
		}
		command[2]=(y>>8) & 0xff;
		command[3]=y & 0xff;
}

void MBCommand::kameraPosition(uint8_t* command,double z, double x)
{		
	//treba prerobit cez define 8.4.2016 prerobene
	/*double MBZISK1=12.0;
	double MBNULPOL1=1811;
	double MBZISK2=11.584;
	double MBNULPOL2=3495;*/
		int yy;
		if(z>=-120 && z<=180){ // platny vstup
			yy=((int) (((int) z*MBZISK1)+MBNULPOL1));
   		} 
		else{
			yy=0;
		}
		
		command[0]=(yy>>8) & 0xff;
		command[1]=yy & 0xff;
	
		if(x>=-30 && x<=90){ // platny vstup
			yy=((int) (((int)-x*MBZISK2)+MBNULPOL2));
		} 
		else{
			yy=0;
		}

		command[2]=(yy>>8) & 0xff;
		command[3]=yy & 0xff;
}
/*** Trieda MCBCommand  (pre Motor control board aj lavy aj pravy)  dedi metody z  triedy Command***/
//public
//constructor
MCBCommand::MCBCommand(uint8_t adresa) : Command(adresa)
{
//init pre motory
	Motor_set.Speed=0;		///@todo treba preradit na radiany
	Motor_set.GearPosition=0;
	pthread_mutex_init(&MCB_mutex, NULL);
	Motor_set.MotorControl=false;
	Motor_set.Clr_I2C_CER=false;
	Motor_set.Clr_RS_CER=false;
	Motor_set.ManualShift=false;

	Motor_set.SpeedSTABF=0;		///neosadene
	Motor_set.SpeedSTABR=0;		///neosadene

	Motor_set.regulator.PH=0;			///proporcionalna zlozka regulatora otacok (H zlozka)
	Motor_set.regulator.PL=10;			///proporcionalna zlozka regulatora otacok (L zlozka)

	Motor_set.regulator.IH=0;			///integralna zlozka regulatora otacok (H zlozka)
	Motor_set.regulator.IL=15;			///integralna zlozka regulatora otacok (L zlozka)
	
	if (adresa == 0x05)
	{
	Motor_set.phs=190;			///poloha serva radenia pre prevod 1:1
	Motor_set.pn=144;			///poloha serva radenia pre neutral
	Motor_set.pls=86;			///poloha serva radenia pre prevod 2:1
	
	}
	else if (adresa == 0x06)
	{
	Motor_set.phs=186;			///poloha serva radenia pre prevod 1:1
	Motor_set.pn=144;			///poloha serva radenia pre neutral
	Motor_set.pls=90;			///poloha serva radenia pre prevod 2:1
		}

	current_vel_set = 0;
	Motor_set.commandID = REQUEST_COMMAND_FLAG;

}
//getery
int MCBCommand::getControlCommand(uint8_t* command){
	
	unsigned char crc;
	command[0] = adresa;
	command[1] = 0x13;
	command[2] = 0x80;
	pthread_mutex_lock(&MCB_mutex);
	//Motor_set.Speed=rychlost;		///!@todo tu treba urobit prepocet z radianov na prislusnu jednotku
	if(adresa == 0x05){
		uint16ToChar2B((-(short)(Motor_set.Speed)), command[4], command[5]);
	}
	else{
		uint16ToChar2B(((short) Motor_set.Speed), command[4], command[5]);
	}
	command[6]=Motor_set.GearPosition;
	command[7]=Motor_set.MotorControl+2*Motor_set.Clr_I2C_CER+4*Motor_set.Clr_RS_CER+8*Motor_set.ManualShift;

	//nulovanie priznakov
	if(Motor_set.Clr_I2C_CER) Motor_set.Clr_I2C_CER=false;
	if(Motor_set.Clr_RS_CER) Motor_set.Clr_RS_CER=false;	
	
	//stabilizator 
	command[8]=Motor_set.SpeedSTABF;
	command[9]=Motor_set.SpeedSTABR;
	command[10]=0;//rezerva
	command[12]=Motor_set.regulator.PH;
	//fprintf(stdout, "PH %X\n", MCBCC0.PH);
	command[13]=Motor_set.regulator.PL;
	//fprintf(stdout, "PL %X\n", MCBCC0.PL);
	command[14]=Motor_set.regulator.IH;
	//fprintf(stdout, "IH %X\n", MCBCC0.IH);
	command[15]=Motor_set.regulator.IL;
	//fprintf(stdout, "IL %X\n", MCBCC0.IL);
	command[16]=Motor_set.phs; //horny prevod 1:1 
	command[17]=Motor_set.pn; //neutral
	command[18]=Motor_set.pls;//dolny prevod 1:2
	Motor_set.commandID = REQUEST_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
	processParity(&command[3]);
	processParity(&command[11]);
	computeCRCWithParity(command,controlCommandLength);
	
	return 6;
}

int MCBCommand::getPartialCommand(uint8_t* command){ //ciastocny command v datasheete uvadzani ako spojene citanie so zapisom, vyuzitelne pre zadavanie rychlosti motora a radenie
	unsigned char crc;
	
	command[0] = adresa;
	command[1] = 0x0B;
	command[2] = 0x01;
	pthread_mutex_lock(&MCB_mutex);
	uint16ToChar2B(((short) Motor_set.Speed), command[4], command[5]);
	command[6]=Motor_set.GearPosition;
	command[7]=Motor_set.MotorControl+2*Motor_set.Clr_I2C_CER+4*Motor_set.Clr_RS_CER+8*Motor_set.ManualShift;
	//nulovanie priznakov
	if(Motor_set.Clr_I2C_CER) Motor_set.Clr_I2C_CER=false;
	if(Motor_set.Clr_RS_CER) Motor_set.Clr_RS_CER=false;	

	command[8] = Motor_set.SpeedSTABF;
	command[9] = Motor_set.SpeedSTABR;
	Motor_set.commandID = REQUEST_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
	command[10] = 0;

	processParity(&command[3]);
	computeCRCWithParity(command, partialCommandLength);
	
	return 26;
}


  void MCBCommand::getStructMotor(ROBLL_SET_MOTOR* Motor)
  {
	pthread_mutex_lock(&MCB_mutex);
  Motor->Speed = Motor_set.Speed;			//!<posielana ziadana rychlost v imulzoch/10ms @todo preratat na ziadanu na rad/s
  Motor->GearPosition = Motor_set.GearPosition;

  Motor->MotorControl = Motor_set.MotorControl;		//!< otackova=0, PWM=true riadenie
  Motor->Clr_I2C_CER = Motor_set.Clr_I2C_CER;		//!< nulovanie priznaku i2c error
  Motor->Clr_RS_CER = Motor_set.Clr_RS_CER;		//!< nulovanie priznaku rs422 error
  Motor->ManualShift = Motor_set.ManualShift;		//!< true=manualne radenie, 0=kontrolovane radenie
  Motor->SpeedSTABF = Motor_set.SpeedSTABF;
  Motor->SpeedSTABR = Motor_set.SpeedSTABR;

		/// regulatory rychlosti
  Motor->regulator = Motor_set.regulator;
	///poloha serva prevodovky
  Motor->phs = Motor_set.phs;
  Motor->pn = Motor_set.pn;
  Motor->pls = Motor_set.pls;
	pthread_mutex_unlock(&MCB_mutex);
}
bool MCBCommand::getMotorControl() //ak je true tak je PWM ak je false tak je PID
   {
	pthread_mutex_lock(&MCB_mutex);
	bool ret = Motor_set.MotorControl;
	pthread_mutex_unlock(&MCB_mutex);
 	return ret;  
}
short MCBCommand::getMotorSpeed()
   {
	pthread_mutex_lock(&MCB_mutex);
	short ret = (short)Motor_set.Speed;
	pthread_mutex_unlock(&MCB_mutex);
	return ret;
  }
  //settery
// TODO
void MCBCommand::setMotorSpeed(double speed)
{
	if(current_vel_set!=speed){
		current_vel_set = speed;
		pthread_mutex_lock(&MCB_mutex);
		Motor_set.Speed = speed;
		Motor_set.commandID |= PARTIAL_COMMAND_FLAG;
		pthread_mutex_unlock(&MCB_mutex);
	}
}
void MCBCommand::setGearPosition(uint8_t position)
{
	pthread_mutex_lock(&MCB_mutex);
	Motor_set.GearPosition = position;
	Motor_set.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
}
void MCBCommand::setManualRad(bool parameter)
{
	pthread_mutex_lock(&MCB_mutex);
	Motor_set.ManualShift = parameter;
	Motor_set.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
}

void MCBCommand::setErrFlags(bool I2C, bool RS422)
{
	pthread_mutex_lock(&MCB_mutex);
	Motor_set.Clr_I2C_CER=I2C;
	Motor_set.Clr_RS_CER=RS422;
	Motor_set.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
}

void MCBCommand::setRegulatorPID(REGULATOR_MOTOR regulator)
{
	pthread_mutex_lock(&MCB_mutex);
	Motor_set.regulator = regulator;
	Motor_set.commandID |= CONTROL_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
}

void MCBCommand::setMotorControl(bool parameter) //false = PID, true = PWM
{
	pthread_mutex_lock(&MCB_mutex);
	Motor_set.MotorControl = parameter;
	Motor_set.commandID |= PARTIAL_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);

}
//getery
int MCBCommand::getCommandID(){
	int id;
	pthread_mutex_lock(&MCB_mutex);
	id = Motor_set.commandID;
	pthread_mutex_unlock(&MCB_mutex);
	return id;
}

int MCBCommand::getCommandID(bool nuluj){
	int id;
	pthread_mutex_lock(&MCB_mutex);	
	id = Motor_set.commandID;
	if(nuluj)
		Motor_set.commandID = REQUEST_COMMAND_FLAG;
	pthread_mutex_unlock(&MCB_mutex);
	return id;
}

/*** Trieda pre Sensor Board ***/
SBCommand::SBCommand(uint8_t adresa) : Command(adresa)
{
		pthread_mutex_init(&SB_mutex, NULL);
    sb_req.adresa = adresa;
    sb_req.dlzka_spravy = 0x83;
    sb_req.cislo_commandu = 0x45;
   
    uint8_t crc = computeCRC((uint8_t*)&sb_req,3);
	sb_req.crcL=crc & 0x0f;
	sb_req.crcH=(crc & 0xf0)>>4;

	if(!computeParity((uint8_t)sb_req.crcL)) sb_req.crcL ^=0x80;
	if(!computeParity((uint8_t)sb_req.crcH)) sb_req.crcH ^=0x80;
	
	sum_samples = 0;
	
	 kompas.SOFT_REV_NUM = 0;
         kompas.COM_BEAR_BYTE = 0;
        kompas.COM_BEAR_WORD = 0;
        kompas.SEN1_DIFSIG = 0;
        kompas.SEN2_DIFSIG = 0;
        kompas.CALVALUE1 = 0;
        kompas.CALVALUE2 = 0;
	kompas.BEAR_AVG = 0;		///pocitana hodnota, klzavy priemer - len ked robot stoji, pre riadenie treba pouzivat COM_BEAR_WORD, ked vobec 
	kompas.BEAR_AVG_COUNT = 0;
}

 int SBCommand::getRequestKompasCommand(uint8_t* command)
 {
			pthread_mutex_lock(&SB_mutex);
		 	command[0] = sb_req.adresa;
    	command[1] = sb_req.dlzka_spravy;
    	command[2] = sb_req.cislo_commandu;
    	command[3] = sb_req.crcH;
    	command[4] = sb_req.crcL;
			pthread_mutex_unlock(&SB_mutex);
    	//printf("%d",requestCommandLength); //komentovane 29.1.2016 Matej Bartosovic (neodkomentovat??)
    	return 14;
}


 int SBCommand::getControlKompasCommand(uint8_t* command, bool calibrate, bool reset)
 {
 	
	command[0] = 0x09; 
	command[1] = 0x85; 
	command[2] = 0x04;
	command[3] = 0x80;
	pthread_mutex_lock(&SB_mutex);
	command[4]=1*reset + 2*calibrate;
	pthread_mutex_unlock(&SB_mutex);
	if(!computeParity(command[4])){ 
		command[4] ^=0x80; 
		command[3]=0x01; 
	}
	computeCRCWithParity(command,7);
	
    	return 6;
}

//odpoved na request command 0x45 (kratsia)
void SBCommand::addSample(uint8_t data[])
{
	 SENZOR_BOARD_ANSWER_STATUS *p;
	  p=(SENZOR_BOARD_ANSWER_STATUS*)data;
	// ROBLL_COMPASS_CMPS03 rob_compass_CMPS03;
	// ROBLL_COMPASS_HM55B rob_compass_HM55B;
	//SensorB_OUT_54 *p;

	kompas.COM_BEAR_WORD = char2BToInt16(data[5], data[6]);
	//rob_compass_CMPS03.COM_BEAR_WORD=((uint16) RobLLRS422ComputeSwb(p->HLRCA.CBW));
	//(*rob_compass_CMPS03).new_data=true;
//	(*rob_compass_CMPS03).time=*rob_time_counter;

	//if(rob_odometry->movement_right || rob_odometry->movement_left){
		///robot je v pohybe - do BEAR_AVG sa da aktualna hodnota komapsu
	//	rob_compass_CMPS03->BEAR_AVG=rob_compass_CMPS03->COM_BEAR_WORD;
	//	rob_compass_CMPS03->BEAR_AVG_COUNT=0;
	//	rob_CMPS03_compute.sum_samples=((double) 0.0);
	//}
	
	///robot stoji - teda aspon vzhladom na pohony - moze sa ratat priemer
		pthread_mutex_lock(&SB_mutex);
	kompas.BEAR_AVG_COUNT++;
	if (kompas.BEAR_AVG_COUNT > 5) //vzorky sa zacnu pridavat az po 5 citaniach
	{	
		sum_samples+=kompas.COM_BEAR_WORD;
		kompas.BEAR_AVG=sum_samples/(kompas.BEAR_AVG_COUNT - 5);
	}
	pthread_mutex_unlock(&SB_mutex);
	
	//rob_compass_HM55B.X_AXIS=((int) RobLLRS422ComputeSwb(p->HLRCA.XM));
	//rob_compass_HM55B.Y_AXIS=((int) RobLLRS422ComputeSwb(p->HLRCA.YM));
	//rob_compass_HM55B.ARM=((double) (((short)RobLLRS422ComputeSwb(p->HLRCA.AP)+ARMP2)*ARMP1));
	//rob_compass_HM55B.new_data=true;
	//rob_compass_HM55B.time=*rob_time_counter;
	
	
	
	//ROS_INFO("CMPS03 uhol svet ( bear word) %.4f po deleni %.4f po deleni  %.4f",rob_compass_CMPS03.COM_BEAR_WORD,rob_compass_CMPS03.COM_BEAR_WORD/180.04,rob_compass_CMPS03.COM_BEAR_WORD/360.08);
	//ROS_INFO("HM55B X %d",rob_compass_HM55B.X_AXIS);
	//ROS_INFO("HM55B Y %d",rob_compass_HM55B.Y_AXIS);
	//ROS_INFO("HM55B ARM %.4f",rob_compass_HM55B.ARM);
	
}

void SBCommand::resetSamples()
{
	pthread_mutex_lock(&SB_mutex);
	sum_samples = 0;
	kompas.BEAR_AVG_COUNT = 0;
	kompas.BEAR_AVG = 0;
	pthread_mutex_unlock(&SB_mutex);
}

double SBCommand::getBearing()
{ 
	pthread_mutex_lock(&SB_mutex);
	double x=kompas.BEAR_AVG/10+90; //kompas je otoceny (rotuje opacne) a potoceny o 90 stupnov
	pthread_mutex_unlock(&SB_mutex);
	if(x>=360)
		x-=360;
	x=360-x;
	
	return x; //lebo datasheet compas cmps03
}

/*** Hlavna trieda Commnad ***/
Command::Command(uint8_t adresa)
{
  this->adresa = adresa;
   req.adresa = adresa;
    req.dlzka_spravy = 0x83;
    if (adresa == 0x09)
    	req.cislo_commandu = 0xC4;
    else
    	req.cislo_commandu = 0x40; 
    uint8_t crc = computeCRC((uint8_t*)&req,3);
	req.crcL=crc & 0x0f;
	req.crcH=(crc & 0xf0)>>4;

	if(!computeParity((uint8_t)req.crcL)) req.crcL ^=0x80;
	if(!computeParity((uint8_t)req.crcH)) req.crcH ^=0x80;
 }
 // private pre SB
 int16_t SBCommand::char2BToInt16(unsigned char H, unsigned char L)
{
	uint16_t result = (H << 8) | L;
	return (int16_t) result;
}
 //public gettery
 //request command je skoro zhodny pre oba moduly preto je implementovany v tejto triede
 int Command::getRequestCommand(uint8_t* command)
 {
 	command[0] = req.adresa;
    	command[1] = req.dlzka_spravy;
    	command[2] = req.cislo_commandu;
    	command[3] = req.crcH;
    	command[4] = req.crcL;
    	//for (int i = 0; i < 5; i++)
    	//	ROS_ERROR("%X", command[i]);
    	//printf("%d",requestCommandLength); //komentovane 29.1.2016 Matej Bartosovic (neodkomentovat??)
    	if (adresa == 0x03)
    	return 34;
    	else if (adresa == 0x09)
    	return 30;
    	else return 26;
}

    //protected pomocne metody pre vypocty
    void Command::computeCRCWithParity(uint8_t *command,int length)
    {
   uint8_t crc=computeCRC((uint8_t *)command,length - 2); //dlzka commandu pri pouzity control command,
   						      // nemozem pouzivat sizeof(command) lebo prijaty command moze mat  vacsiu dlzku
	//crc L
	command[length - 1]=crc & 0x0f;
	if(!computeParity(command[length - 1])) command[length - 1] ^=0x80;
	//crc H
	command[length - 2]=(crc & 0xf0)>>4;
	if(!computeParity(command[length - 2])) command[length - 2] ^=0x80;
    }
    
    void Command::processParity(uint8_t *byte){
	int i,j;
	uint8_t parbajt=0;
	for(i=1;i<8;i++){
		uint8_t znak=*(byte+i);
		j=computeParity(znak);
		if(!j){
			parbajt |= 0x80;
			*(byte+i) ^= 0x80;       // zmen paritu
		}
		parbajt=parbajt >> 1;
	}
	j=computeParity(parbajt);
	if(!j){
		parbajt ^= 0x80;             // zmen paritu
	}
	*(byte+0) = parbajt;
}

    int Command::computeParity(uint8_t x){
	x ^= x >> 1;
	x ^= x >> 2;
	x ^= x >> 4;
	return x & 1 ;
}

    uint8_t Command::computeCRC(uint8_t *buffer, unsigned int num){
	unsigned int ind = 0;
	uint8_t CRC = 0x00;

	while((ind<num)){
        	CRC = har_tableCRC[CRC ^ (*buffer++)];
        	ind++;
	}

	return CRC;
}

void Command::uint16ToChar2B(uint16_t X, uint8_t &H, uint8_t &L){
	H= (X>>8) & 0xff; 
	L= X & 0xff;
}
