#ifndef COMMAND_H_
#define COMMAND_h_

#include <stdio.h>
#include <stdint.h>
#include <ros/ros.h>


#define MBOFS1 50.0
#define MBOFS2 50.0
#define MBZISK1 12.0
#define MBNULPOL1 1811
#define MBZISK2 11.584
#define MBNULPOL2 3495

//todo zmazat
double read_kompas();

//TODO skovat do classy command
const static int PARTIAL_COMMAND_FLAG = 4;
const static int CONTROL_COMMAND_FLAG = 2;
const static int UNITED_COMMAND_FLAG  = 1;
const static int REQUEST_COMMAND_FLAG = 0;

typedef struct request_command
	   {
		uint8_t adresa;
		uint8_t dlzka_spravy;
		uint8_t cislo_commandu;
		uint8_t crcH;
		uint8_t crcL;
	   } REQUEST_COMMAND;

	typedef struct robll_set_mainb {		// MB odomna pre ronyho - len SS
		///pm1
		bool MCBsSB_5V;
		bool MCBs_12V;
		bool videoTransmitter;
		bool wifi;
		bool laser;
		bool GPS;
		bool ARM_5V;
		bool ARM_12V;
		///pm2
		bool PC2;
		bool kamera;
		///ctrl
		bool centralStop;
		bool powerOff;
		bool fullBatAck;
		bool resetQbat;
		bool video1;
		bool video2;
		///cam 
		bool posRotCam;		//!< ak je true tak je polohova regulacia inak otackova
		int  camAngleX;
		int  camAngleZ;
		///premenne regulacie - P a I regulator kamery
		uint8_t pko;
		uint8_t pkk;
		uint8_t iko;
		uint8_t ikk;
		int commandID;
	} SET_MAIN_BOARD;
	

	typedef struct regulator_motor{

		/// regulatory rychlosti
			uint8_t PH;
			uint8_t PL;

			uint8_t IH;
			uint8_t IL;

	}REGULATOR_MOTOR;
typedef struct robll_set_motor { // MCB odomna pre ronyho - pouzijem len v sstatusa SS
		double Speed;			//!<posielana ziadana rychlost v imulzoch/10ms @todo preratat na ziadanu na rad/s
		unsigned char GearPosition;

		bool MotorControl;		//!< otackova=0, PWM=true riadenie
		bool Clr_I2C_CER;		//!< nulovanie priznaku i2c error
		bool Clr_RS_CER;		//!< nulovanie priznaku rs422 error
		bool ManualShift;		//!< true=manualne radenie, 0=kontrolovane radenie
	
		unsigned char SpeedSTABF;
		unsigned char SpeedSTABR;

		REGULATOR_MOTOR regulator;

		///poloha serva prevodovky
		unsigned char phs;
		unsigned char pn;

		unsigned char pls;
		int commandID;

		//bool init;			//!< ak je true este neboli naplnene pociatocne udaje
	} ROBLL_SET_MOTOR;
	
typedef struct har_rca_smstat {
		unsigned short CBW,XM,YM,AP;
	} HAR_RCA_SMSTAT;
	
typedef struct har_res_rca_54 {
		unsigned char f1,f2,addr,dlzka,cislocc;
  		HAR_RCA_SMSTAT HLRCA;
		unsigned char CRC;
	} SENZOR_BOARD_ANSWER_STATUS;
	
typedef struct robll_compass_CMPS03{
        int SOFT_REV_NUM;
        double COM_BEAR_BYTE;
        double COM_BEAR_WORD;
        int SEN1_DIFSIG;
        int SEN2_DIFSIG;
        int CALVALUE1;
        int CALVALUE2;
	double BEAR_AVG;		///pocitana hodnota, klzavy priemer - len ked robot stoji, pre riadenie treba pouzivat COM_BEAR_WORD, ked vobec 
	unsigned long  BEAR_AVG_COUNT;		///pocet vzoriek z ktorych bol vypocitany priemer
	//bool new_data;			//!< su data nove
	//ros::Time time;			//!< cas ziskania dat
} COMPASS_CMPS03;
	   
static unsigned char har_tableCRC[256]={
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53 
};

class Command
{
	public:  
	Command(uint8_t adresa);
	int getRequestCommand(uint8_t* command);
	const static int requestCommandLength = 5;
	protected:
	  uint8_t adresa;
	  uint8_t computeCRC(uint8_t *buffer, unsigned int num);
	  int computeParity(uint8_t x);
	  void processParity(uint8_t *byte);
	 void uint16ToChar2B(uint16_t X, uint8_t &H, uint8_t &L);
	 void computeCRCWithParity(uint8_t *command,int length);

	 private:
	  REQUEST_COMMAND req;
	
};

class MBCommand: public Command
{
	public:
	MBCommand(uint8_t adresa);
	int getControlCommand(uint8_t* command);
	int getPartialCommand(uint8_t* command);
	int getUnitedCommand(uint8_t* command);
	bool getPosRotCam();
	void getStructMain(SET_MAIN_BOARD* MB);
	int getCommandID();
	int getCommandID(bool nuluj);
	bool getCentralStop();
	void setParamatersMB_PM(bool MCBsSB_5V,bool MCBs_12V, bool videoTransmitter, bool wifi, bool laser,
    				  bool GPS, bool ARM_5V, bool ARM_12V, bool PC2, bool kamera);
	void setParamatersMB(SET_MAIN_BOARD* config);
    void setParametersMB_CTRL(bool centralStop, bool powerOff, bool fullBatAck, bool resetQbat, bool camera_source);
    void setMCBsSB_5V(bool parameter);
    void setMCBs_12V(bool parameter);
    void setVideoTransmitter(bool parameter);
    bool switchVideo();
 	void setWifi(bool parameter);
	void setLaser(bool parameter);
	void setGPS(bool parameter);
 	void setCentralStop(bool parameter);
	void setPowerOff(bool parameter);
	void setKamera(bool parameter);
	void setArm5V(bool parameter);
	void setArm12V(bool parameter);
	void setArmPower(bool parameter);
	void setPC2(bool parameter);
	void setPosRotCam(bool parameter);
	void setFullBatery(bool parameter);
	void setResetQBatery(bool parameter);
	void resetBatery();
	void setVideoGrabberSwitch(bool parameter);
	void setVideoTransmitterSwitch(bool parameter);
  	void setKameraRegulator(uint8_t pko,uint8_t pkk,uint8_t iko, uint8_t ikk);
  	void setKameraCommand(int Z, int X);
	const static int controlCommandLength = 21;  //zapisuje vsetko aj parametre PI regulatora kamery
	const static int partialCommandLength = 13;   //nezapisuje paramtre PI regulatora a dostava odpoved okrem  info napajani zariadeni
	const static int unitedCommandLength = 21;    //zapisuje vsetko + dostava odpoved na vsetko
	private: 
	SET_MAIN_BOARD rob_set_MB;
	pthread_mutex_t MB_mutex;
	void kameraVelocity(uint8_t* command,double x, double z);
	void kameraPosition(uint8_t* command,double z, double x);
	int last_X_camera;
	int last_Z_camera;
};

class MCBCommand: public Command
{
	public:
	MCBCommand(uint8_t adresa);
	int getControlCommand(uint8_t* commnand);
    int getPartialCommand(uint8_t* command);
    void getStructMotor(ROBLL_SET_MOTOR* Motor);
    bool getMotorControl();
    short getMotorSpeed();
    int getCommandID();
    int getCommandID(bool nuluj);
    void setMotorSpeed(double speed);
  	void setManualRad(bool parameter);
  	void setGearPosition(uint8_t position);
  	void setErrFlags(bool I2C, bool RS422);
  	const static int controlCommandLength = 21;  //prikaz nema odpoved status, v prikaze je mozne zapisovat aj parametre PI regulatora
	const static int partialCommandLength = 13;	//prikaz ma aj odpoved status: (enkodery, napatie, flagy, atd)


	void setRegulatorPID(REGULATOR_MOTOR regulator);
	void setMotorControl(bool parameter); //otackova regulacia 0 PWM 1
  	private:
  	 ROBLL_SET_MOTOR Motor_set;
		 pthread_mutex_t MCB_mutex;
	double current_vel_set;
};

class SBCommand: public Command
{
	public:
	SBCommand(uint8_t adresa);
	int getRequestKompasCommand(uint8_t* command);
	int getControlKompasCommand(uint8_t* command, bool calibrate, bool reset);
	void addSample(uint8_t* data);
	void resetSamples();
	double getBearing();
	/*int getControlCommand(uint8_t* commnand);
    	int getPartialCommand(uint8_t* command);
    	void getStructMotor(ROBLL_SET_MOTOR* Motor);
    	bool getMotorControl();
    	void setMotorSpeed(double speed);
  	void setManualRad(bool parameter);
  	void setGearPosition(uint8_t position);
  	void setErrFlags(bool I2C, bool RS422);
  	const static int controlCommandLength = 21;
	const static int partialCommandLength = 13;
	void setRegulatorPID(uint8_t PH,uint8_t PL,uint8_t IH,uint8_t IL);
	void setMotorControl(bool parameter); //otackova regulacia 0 PWM 1*/
  	private:
  	int16_t char2BToInt16(unsigned char H, unsigned char L);
  	  REQUEST_COMMAND sb_req;
  	  COMPASS_CMPS03 kompas; 
  	  double sum_samples;
			pthread_mutex_t SB_mutex;
  	  
};

#endif //COMMAND_H_
