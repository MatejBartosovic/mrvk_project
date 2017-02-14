#include <mrvk_driver/Mb_status.h>
#include <mrvk_driver/Mcb_status.h>
#include <boost/thread.hpp>



#define QBAT 12000000.0
#define MBB12 0.008042
#define MBB1 0.004612
#define MBIB 0.0092929 // prud z baterie
#define MBT1 0.5
#define MCBIGM 0.0007446
#define MCB5V 0.005616
#define MCBUM 0.03742
#define MCBIM 0.01588

#define MBZISK1 12.0
#define MBNULPOL1 1811
#define MBZISK2 11.584
#define MBNULPOL2 3495

#define ODOMETRY_CONSTANT 0.006546076617684

class Conversions{
public:

	Conversions();
	mrvk_driver::Mb_status getStatusMB();
	mrvk_driver::Mcb_status getStatusMCB(uint8_t device);

	//return velocity in rad/s
	double getVelLeftWheel();
	double getVelRightWheel();

	//return position in rad/s
	double getPosLeftWheel();
	double getPosRightWheel();

	//return velocity in rad/s (only gear 1:1)
	double getVelLeftMotor();
	double getVelRightMotor();

	//return position in rad/s (only gear 1:1)
	double getPosLeftMotor();
	double getPosRightMotor();

	//return position in rad/s
	double getCameraPositionZ();
	double getCameraPositionX();

    template <typename TMember>
	auto getStatus(TMember member){
		boost::unique_lock<boost::mutex> lock(data_mutex);
        return statusMB.*member;
	}

    //example getStatus(&mrvk_driver::Mb_status::central_stop,true);

    bool getPowerArm();
	//mcb status
	bool getStatusMotorErrors();

	//const static int HEADER_LENGTH = 4;

	const static uint8_t HEADER = 0xFF; //byte 0,1

	//byte 4 ANSWER_ID
	const static uint8_t MSG_OK = 0xFE;
	const static uint8_t MSG_ERROR = 0xFF;

	//byte 2 DEVICE_ADRESS
	const static uint8_t LEFT_MOTOR_ADRESS = 0x05;
	const static uint8_t RIGHT_MOTOR_ADRESS = 0x06;
	const static uint8_t MAIN_BOARD_ADRESS = 0x03;


	//poradie bytov v hlavicke odpovede
	const static uint8_t HEADER0 = 0;
	const static uint8_t HEADER1 = 1;
	const static uint8_t DEVICE_ADRESS = 2;
	const static uint8_t ANSWER_LENGTH = 3;
	const static uint8_t ANSWER_ID = 4;
	const static uint8_t ERR_CODE = 5;

	static int16_t char2BToInt16(unsigned char H, unsigned char L);
	static uint32_t char4BToUint32(uint8_t HH, uint8_t H, uint8_t L, uint8_t LL);


protected:
	void convertMsg(uint8_t *data, uint8_t device);
    boost::mutex data_mutex;

private:
	void answerMB_40_02(uint8_t *data);
	void answerMB_10(uint8_t *data);

	void answerMCB(uint8_t *data, uint8_t device);

	int velWheels[2];
	int velActuators[2];

	uint16_t posWheels[2];
	uint16_t posActuators[2];

	double cameraPositionX;
	double cameraPositionZ;

	mrvk_driver::Mb_status statusMB;
	mrvk_driver::Mcb_status statusMCB[2]; //0 - lavy a 1 - pravy

};

