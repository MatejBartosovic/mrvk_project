#ifndef COMMUNICATION_INTERFACE_H_
#define COMMUNICATION_INTERFACE_H_

#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>


class CommunicationInterface{
public:

//    typedef enum {
//        MainBoardFlag = 1,
//        LeftMotorBoardFlag = 2,
//        RightMotorBoardFlag = 4,
//        MotorBoardsFlag = 6,
//        AllDevicesFlag = 7
//    }StatusFlags;
//
//	const static int MAIN_BOARD = 0;
//	const static int LEFT_MOTOR = 1;
//	const static int RIGHT_MOTOR = 2;
//
	CommunicationInterface(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize);
	~CommunicationInterface();
//
//	bool init();
//	bool isActive();
//	void close();
//
//	bool write();
//	int waitToRead();
//
//	void setMotorParameters(REGULATOR_MOTOR regulator, bool regulation_type);
//
//	void setCameraPosition(double linearX, double angularZ);
//	void setMotorsVel(double left_vel,double right_vel);
//	void blockMovement(bool param);
//
//    bool getWriteStatus(StatusFlags flags, boost::unique_lock<boost::mutex>& mutex);
//    bool getNewDataStatus(StatusFlags flags);
//
//	MCBCommand* getMotorControlBoardLeft();
//	MCBCommand* getMotorControlBoardRight();
//	MBCommand* getMainBoard();
//
//	//command mutex
//	boost::mutex write_mutex;
private:

//	void setupPort(int sb,int p,int bs);
//
//	int write(int id,uint8_t *dataWrite, int lengthWrite);
//	bool read(int lengthRead);
//	void setFds(fd_set *set,int* flag);
//	int getReadableFd(fd_set *set,int* flag);
//	int writeMB();
//	int writeMotors();
//
//	//MOTOR BOARD
//    int leftMCBStatusUpdate();
//    int rightMCBStatusUpdate();
//    int leftSendPartialCommand();
//    int rightSendPartialCommand();
//    int leftSendControlCommand();
//    int rightSendControlCommand();
//
//	//MAIN BOARD
//    int MBStatusUpdate();
//    int MBSendPartialCommd();
//    int MBSendControlCommd();
//    int MBSendUnitedCommd();
//
//	std::vector<serial::Serial*> my_serials;
//
//    //command generation classes
//	MCBCommand lavy;
//	MCBCommand pravy;
//	MBCommand mb;
//
//    //communication config variables
//    bool active;
//    bool blocked;
//	std::vector<std::string> ports;
//	int fd_max;
//	std::vector<int> read_lengths;
//	int baud;
//	int byteSize;
//	int stopBits;
//	int parity;
//
//    //sinchronization variables
//    int new_data_status;
//    int write_status;
//
//    boost::mutex new_data_murex;
//
//    boost::condition_variable write_complete;
//    boost::condition_variable new_data;
};


#endif /* COMMUNICATION_INTERFACE_H_ */

