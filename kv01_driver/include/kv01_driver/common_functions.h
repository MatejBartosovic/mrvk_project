#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<sstream>
#include<iostream>
#include "ros/ros.h"

#define PHI 			3.14159265359

// data pre prikazy unknow command / OK a time out
#define OK_data1 0x4F
#define OK_data2 0x4B

#define TO_data1 0x54
#define TO_data2 0x4F

#define UC_data1 0x55
#define UC_data2 0x43



//prikazy na posielanie pre servis Driver spolocne s datami 
#define comm_status  		0xAA

#define comm_brzdy		0xBB
#define comm_brzdy_run		0xB1
#define comm_brzdy_stop 	0xB0

#define comm_gripper_control    0xD5

#define comm_voltages 		0xCC

#define comm_message		0xDD

  #define comm_motory_en	  0xD1
  #define comm_motory_dis	  0xD0
  #define comm_motory_speed       0xD2
  #define comm_position           0xD3
  
#define comm_err_ok	 	0x11
#define comm_err_communication 	0x22
#define comm_err_command       	0x33 
#define comm_err_data       	0x44 

#define comm_central_stop_wifi	0xFF

#define comm_response_ok	0x11
#define comm_response_bad	0x00


//Funkcie

#ifndef common_functions_H_INCLUDED
#define common_functions_H_INCLUDED


// zadefinovanie struktury pre inicializaciu premennych motora
typedef struct har_arm_machine_data {	// konstatny na prepocty KK
	short offset_j[6];
	
	double Ulimit_j[6];
	double Dlimit_j[6];
	double Acclimit_j[6];			///limit v rad/s
	double velocityConst_j[6];		///prepoctova konstanta z uhlovej rychlosti ramena na ot/min pre motor
	double positionConst_j[6];		///prepoctova konstanta z rad na impulzy pre faulhaber
	
	int maxvel_j[6];			///v ot/min pre motor
	double maxvel_rad_j[6];			///v rad/s pre klb
	
	uint8_t commAddress_j[6];		///adresy pohonov ramenoa

	double voltage24Vconst;
	double voltage12Vconst;
	double voltage5Vconst;

	double angle_const;
	double angle_const_phi;
	double angle_const_dec;
	} HAR_ARM_MACHINE_DATA;
	
	// struktura na kontrolu dat 
	typedef struct data_ok_command{
	  uint8_t address;
	  uint8_t size;
	  uint8_t message;
	  uint8_t data1;	
	  uint8_t data2;
	  uint8_t crc;
	  uint8_t end;
	} DATA_OK_COMMAND;   
	
	// struktura na naplnenie napatiami
	typedef struct robll_data_arm_voltages{
	double volt24V_j;
	double volt12V_j;
	double volt5V_j;
	} ROBLL_DATA_ARM_VOLTAGES; 
	
	//ziadost sprava pre D0
	typedef struct har_arm_req_d0 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t data;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_REQ_D0;
	
	//ziadost sprava pre d1
	typedef struct har_arm_req_d1 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t mode;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_REQ_D1;
	
	//odpoved pre d1
	typedef struct har_arm_res_d1 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t data;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_RES_D1;
	
	
	//struktura pre citaj D4 - struktura pre vypocet napati
	typedef struct har_arm_res_d4 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t volt24H;
		uint8_t volt24L;
		uint8_t volt5H;
		uint8_t volt5L;
		uint8_t volt12H;
		uint8_t volt12L;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_RES_D4;	

	// struktura pre pisanie na D4
	typedef struct har_arm_req_d4 {
		uint8_t address;
		uint8_t length;
		uint8_t command;		///v tomto pripade d4
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_REQ_D4;
		///poslanie prikazu D3 - vycitanie stavu klbu
	typedef struct har_arm_res_d3 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t data1;
		uint8_t data2;
		uint8_t data3;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_RES_D3;
	
	// ziadost sprava pre d3
	typedef struct har_arm_req_d3 {
		uint8_t address;
		uint8_t length;
		uint8_t command;		///v tomto pripade d3
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_REQ_D3;
	
	typedef struct robll_data_arm_status {
		bool timeout_j[6];				///stav komunikacie
		bool brake_j[6];				///stavy brzd
		bool LSSTOP_j[6];				///komunikacny modul nepreposiela hodnoty menicu
		bool limop_j[6];				///obmedzeny rezim menicov
		bool limit_j[6];				///stav koncovych spinacov - j4 vzdy false  a j6 je stav chapadla

		uint8_t grip_load;
		bool grip_phase;
		bool grip_brake;

	//	bool new_dataD0;
		bool new_data_D1;
		bool new_data_D2;
		bool new_data_D3;

		bool new_data;
		unsigned long time;
	} ROBLL_DATA_ARM_STATUS;

	typedef struct robll_data_arm_position {
	///udaje o polohe
		double alpha;
		double beta;
		double gamma;
		double delta;
		double theta;
		double phi;

		double inv_alpha;
		double inv_beta;
		double inv_gamma;
		double inv_delta;
		double inv_theta;
		double inv_phi;

		///udaje o polohe koncoveho bodu - trebapocitat
		double x;
		double y;
		double z;

		///udaje o smerovani chapadla - trebapocitat
		double sx;
		double sy;
		double sz;

	} ROBLL_DATA_ARM_POSITION;
	
	
	typedef struct har_arm_req_d5 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t data;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_REQ_D5;

	typedef struct har_arm_res_d5 {
		uint8_t address;
		uint8_t length;
		uint8_t command;
		uint8_t data;
		uint8_t crc;
		uint8_t end;
	} HAR_ARM_RES_D5;

	
class common_functions
{
private:
  HAR_ARM_REQ_D5 requestD5;
  HAR_ARM_REQ_D4 requestD4;
  HAR_ARM_REQ_D3 requestD3;
  HAR_ARM_REQ_D1 requestD1;
//   
  uint8_t *requestD0;
  DATA_OK_COMMAND *responsedata;
  HAR_ARM_RES_D1 responseD1;

 // ROBLL_DATA_ARM_POSITION	test;
  ROBLL_DATA_ARM_POSITION	rob_arm_position;//=&test1;
  
 // ROBLL_DATA_ARM_STATUS		test;
  ROBLL_DATA_ARM_STATUS 	rob_arm_status;//=&test2;  
  
public:
// ROBLL_DATA_ARM_VOLTAGES	test;//=&test;
HAR_ARM_MACHINE_DATA 	machine_data;
ROBLL_DATA_ARM_VOLTAGES	rob_arm_voltages;

bool odbrzdene[6];
  
common_functions();
~common_functions();
  
bool init_driver();
uint8_t Calculate_CRC(uint8_t *buffer, unsigned int num);
unsigned short Calculate_HL(uint8_t H, uint8_t L);
std::string to_string(int i);

uint8_t* ArmCreateD0(uint8_t data, int joint, double value);
bool    ArmReceiveD0(uint8_t *buffer,int joint,uint8_t data);

uint8_t* ArmCreateD1(uint8_t *message,int joint,uint8_t data);
bool ArmReceiveD1(uint8_t *buffer,uint8_t data, int joint);

uint8_t* ArmCreateD3(uint8_t *message,int joint);
double ArmReceiveD3(uint8_t *buffer,int joint);

uint8_t* ArmCreateD4(uint8_t *message,int joint);
void ArmReceiveD4(uint8_t *buffer);

uint8_t* ArmCreateD5(uint8_t *message,int joint); 



bool check_response(uint8_t* buffer);



};


#endif /* common_functions */

