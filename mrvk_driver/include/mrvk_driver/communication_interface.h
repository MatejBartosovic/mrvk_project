#ifndef COMMUNICATION_INTERFACE_H_
#define COMMUNICATION_INTERFACE_H_

#include <mrvk_driver/command.h>
#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/make_shared.hpp>

#include <mrvk_driver/conversions.h>

class CommunicationInterface : public Conversions{
public:

	const static int MAIN_BOARD = 0;
	const static int LEFT_MOTOR = 1;
	const static int RIGHT_MOTOR = 2;


	CommunicationInterface(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize);
	~CommunicationInterface();
	bool init();
	bool isActive();
	void registerMutex(pthread_mutex_t* mutex,int id);
	bool getCallbackResult(int id);
	int waitToRead();


	//MAIN BOARD
	//jednorazove prikazy
	bool MBStatusUpdate();
	bool MBSendPartialCommd();
	bool MBSendControlCommd();
	bool MBSendUnitedCommd();
	bool setCameraVelocity(double linearX, double angularZ);
	bool setCameraPosition(double linearX, double angularZ);
	bool setMainBoard(SET_MAIN_BOARD *param);
	//prikazy, ktore  sa ulozia do struktury a na ich zapis je potrebne zavolat funkciu writeMB
	bool sendMainBoardStruct();
	MBCommand* getMainBoard();


	//MOTOR BOARD
	bool leftMCBStatusUpdate();
	bool rightMCBStatusUpdate();
	bool leftSendPartialCommand();
	bool rightSendPartialCommand();
	bool leftSendControlCommand();
	bool rightSendControlCommand();
	//TODO pojde prec
	bool setSpeedLeftMotor(double speed);
	//TODO pojde prec
	bool setSpeedRightMotor(double speed);
	//TODO pojde prec
	bool resetFlags();
	void setMotorsVel(double left_vel,double right_vel);
	bool setMotorParameters(REGULATOR_MOTOR regulator, bool regulation_type);
	MCBCommand* getMotorControlBoardLeft();
	MCBCommand* getMotorControlBoardRight();

	//bool resetCentralStop();

	void close();

private:
	void setupPort(int sb,int p,int bs);

	bool isAnswerOk();

	void notifyAllCallbacks(bool response,int id);
	void processCallbacks(int id);

	bool write(int id,uint8_t *dataWrite, int lengthWrite);
	bool read(int lengthRead);
	void setFds(fd_set *set,int* flag);
	int getReadableFd(fd_set *set,int* flag);

	bool active;
	std::vector<serial::Serial*> my_serials;
	MCBCommand lavy;
	MCBCommand pravy;
	MBCommand mb;


	//todo nestaci to ako lokalne premenne??
	std::vector<std::string> ports;
	int fd_max;
	std::vector<int> read_lengths;
	int baud;
	int byteSize;
	int stopBits;
	int parity;
	pthread_mutex_t callbacks_mutex;
	std::vector<pthread_mutex_t*> waitings_callbacks[3];
	std::vector<pthread_mutex_t*> processed_callbacks[3];
	bool callback_res[3];
};

#endif /* COMMUNICATION_INTERFACE_H_ */

