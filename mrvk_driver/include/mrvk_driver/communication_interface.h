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

    const static int MAIN_BOARD_BROADCAST_FLAG = 1;
    const static int LEFT_MOTOR_BROADCAST_FLAG = 2;
    const static int RIGHT_MOTOR_BROADCAST_FLAG= 4;
    const static int MOTORS_BROADCAST_FLAG= 6;
    const static int ALL_BROADCAST_FLAG= 7;


	CommunicationInterface(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize);
	~CommunicationInterface();

	bool init();
	bool isActive();
	void close();

	bool write();
	int waitToRead();

	void setMotorParameters(REGULATOR_MOTOR regulator, bool regulation_type);

	void setCameraPosition(double linearX, double angularZ);
	void setMotorsVel(double left_vel,double right_vel);


	MCBCommand* getMotorControlBoardLeft();
	MCBCommand* getMotorControlBoardRight();
	MBCommand* getMainBoard();

	//TODO daco s timto vymislet
	boost::mutex broadcast_mutex;
    boost::condition_variable broadcast;
    int succes;
private:

	void setupPort(int sb,int p,int bs);

	int write(int id,uint8_t *dataWrite, int lengthWrite);
	bool read(int lengthRead);
	void setFds(fd_set *set,int* flag);
	int getReadableFd(fd_set *set,int* flag);
	int writeMB();
	int writeMotors();

	//MOTOR BOARD
    int leftMCBStatusUpdate();
    int rightMCBStatusUpdate();
    int leftSendPartialCommand();
    int rightSendPartialCommand();
    int leftSendControlCommand();
    int rightSendControlCommand();

	//MAIN BOARD
    int MBStatusUpdate();
    int MBSendPartialCommd();
    int MBSendControlCommd();
    int MBSendUnitedCommd();

	bool active;
	std::vector<serial::Serial*> my_serials;
	MCBCommand lavy;
	MCBCommand pravy;
	MBCommand mb;

	std::vector<std::string> ports;
	int fd_max;
	std::vector<int> read_lengths;
	int baud;
	int byteSize;
	int stopBits;
	int parity;
};


#endif /* COMMUNICATION_INTERFACE_H_ */

