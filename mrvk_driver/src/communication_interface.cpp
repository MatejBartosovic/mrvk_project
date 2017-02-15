#include <mrvk_driver/communication_interface.h>
#include <boost/foreach.hpp>


/*
 *
 *
 * Public functions
 *
 *
 * */


CommunicationInterface::CommunicationInterface(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize):
mb(MAIN_BOARD_ADRESS), pravy(RIGHT_MOTOR_ADRESS), lavy(LEFT_MOTOR_ADRESS),blocked(false){

	if(ports.size()!=3)
		throw std::invalid_argument("Invalid serial ports.");

	read_lengths.resize(3);
	this->ports = ports;
	this->baud = baudrate;
	this->stopBits = stopBits;
	this->byteSize = byteSize;
	this->parity = parity;

	active = false;
}

bool CommunicationInterface::init(){


	if (active)		//ak vsetko bezi v poriadku nemusi sa znova volat init
		return true;

	try{
		BOOST_FOREACH(std::string port,ports){
			my_serials.push_back(new serial::Serial(port, baud, serial::Timeout::simpleTimeout(50)));
		}
	}catch (std::exception& e){
		ROS_ERROR("port sa neotvoril");
		active = false;
		return false;
	}

	std::vector<int> fds;
	BOOST_FOREACH(serial::Serial* serial,my_serials){

		if (serial->isOpen())
			ROS_INFO("port otvoreny");
		else{
			ROS_ERROR("port sa neotvoril");
			active = false;
			return false;
		}

		try{
			setupPort(stopBits, parity, byteSize);
			serial->flush();

		}catch (std::exception& e){
			ROS_ERROR("nenastavili sa parametre portu");
			active = false;
			return false;
		}
		fds.push_back(serial->getFd());
	}
	fd_max = -1;
	BOOST_FOREACH(int fd,fds){
		if(fd>fd_max)
			fd_max = fd;
	}
	active = true;
	ROS_INFO("init done");

		return true;
}

bool CommunicationInterface::isActive(){
	return active;
}

void CommunicationInterface::close(){
	if (active){
		BOOST_FOREACH(serial::Serial* serial,my_serials){
			serial->close();
		}
		active = false;
	}
}

bool CommunicationInterface::write(){

	boost::mutex::scoped_lock locfk(write_mutex);
    write_status =  writeMB();
    write_status |= writeMotors();
	write_complete.notify_all();
}

int CommunicationInterface::waitToRead(){

	fd_set read_set;
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 20000; //200ms
	int readable_fds;
	int sucesfull_readings = 0;
	int index;
	int setFlags = 0;
    new_data_status = 0;        //reset received data status

	while(sucesfull_readings != my_serials.size()){
		setFds(&read_set,&setFlags);
		readable_fds = select(fd_max+1,&read_set,NULL,NULL,&timeout);
		if (readable_fds == -1){
			ROS_ERROR("select failed");
			if(timeout.tv_usec == 0){
                new_data.notify_all();
				return sucesfull_readings;
            }
			else
				continue; //TODO nieco lebsie tu vymysliet
		}
		for(int i = 0;i<readable_fds;i++){
			index = getReadableFd(&read_set,&setFlags); // vrati file descriptor, ktory je citatelny
			if(index !=-1){
				sucesfull_readings ++;
                boost::mutex::scoped_lock lock(new_data_murex);
				read(index);
			}
		}
		if(timeout.tv_usec == 0){
            ROS_ERROR("Receive TIMEOUT flags = '%d'",setFlags);  //if flag is set device roesponded (1 mb, 2 mcbl, 4 mcbr )
			break;
        }
	}
    new_data.notify_all();
	return sucesfull_readings;
}

void CommunicationInterface::setMotorParameters(REGULATOR_MOTOR regulator, bool regulation_type){

	lavy.setRegulatorPID(regulator);
	lavy.setMotorControl(regulation_type);
	pravy.setRegulatorPID(regulator);
	pravy.setMotorControl(regulation_type);

}

void CommunicationInterface::setCameraPosition(double linearX, double angularZ){

	//mb.setPosRotCam(true); // true poloha , false rychlost
	mb.setKameraCommand(linearX/180*M_PI,angularZ/180*M_PI);
	return;
}

void CommunicationInterface::setMotorsVel(double left_vel,double right_vel){

	if (blocked){
		left_vel = 0;
		right_vel = 0;
	}

	lavy.setMotorSpeed(-left_vel/ODOMETRY_CONSTANT_VELOCITY);
	pravy.setMotorSpeed(right_vel/ODOMETRY_CONSTANT_VELOCITY);

}

void CommunicationInterface::blockMovement(bool param){

	blocked = param;
}

//Getters

MBCommand* CommunicationInterface::getMainBoard(){
	return &mb;
}

MCBCommand* CommunicationInterface::getMotorControlBoardLeft(){
	return &lavy;
}
MCBCommand* CommunicationInterface::getMotorControlBoardRight(){
	return &pravy;
}

bool CommunicationInterface::getNewDataStatus( StatusFlags flags){
    boost::mutex::scoped_lock lock(new_data_murex);
    new_data.wait(lock);
    return ((new_data_status & flags) == flags);
}

bool CommunicationInterface::getWriteStatus(StatusFlags flags, boost::unique_lock<boost::mutex>& lock){
    write_complete.wait(lock);
    return ((write_status & flags) == flags);
}

/*
 *
 *
 * Private functions
 *
 *
 * */

int CommunicationInterface::writeMB(){

	int succes = 0;
	switch(__builtin_ffs(getMainBoard()->getCommandID())){
		case REQUEST_COMMAND_FLAG:
            succes |= MBStatusUpdate();
			break;
		case PARTIAL_COMMAND_FLAG-1:
            succes |= MBSendPartialCommd();
			//ROS_ERROR("MB partial");
			break;
		case CONTROL_COMMAND_FLAG:
            succes |= MBSendControlCommd();
			//ROS_ERROR("MB control");
			break;
		case UNITED_COMMAND_FLAG:
            succes |= MBSendUnitedCommd();
			//ROS_ERROR("MB united");
			break;
		default:
			ROS_ERROR("V Command ID Main boardy bola zla hodnota (toto by nikdy nemalo nastat)");
            succes = false;
	}
	return succes;
}

int CommunicationInterface::MBStatusUpdate(){

	uint8_t command[MBCommand::requestCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getRequestCommand(command);
	return write(MAIN_BOARD, command, MBCommand::requestCommandLength);

}

int CommunicationInterface::MBSendPartialCommd(){

	uint8_t command[MBCommand::partialCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getPartialCommand(command);
	return write(MAIN_BOARD, command, MBCommand::partialCommandLength);
}

int CommunicationInterface::MBSendControlCommd(){

	uint8_t command[MBCommand::controlCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getControlCommand(command);
	return write(MAIN_BOARD, command, MBCommand::controlCommandLength);
}

int CommunicationInterface::MBSendUnitedCommd(){

	uint8_t command[MBCommand::unitedCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getUnitedCommand(command);
	return write(MAIN_BOARD, command, MBCommand::unitedCommandLength);
}


int CommunicationInterface::leftMCBStatusUpdate(){

	uint8_t command[MCBCommand::requestCommandLength];
	read_lengths[LEFT_MOTOR] = lavy.getRequestCommand(command);
	return write(LEFT_MOTOR,command,MCBCommand::requestCommandLength);

}

int CommunicationInterface::rightMCBStatusUpdate(){

	uint8_t command[MCBCommand::requestCommandLength];
	read_lengths[RIGHT_MOTOR] = pravy.getRequestCommand(command);
	return write(RIGHT_MOTOR, command, MCBCommand::requestCommandLength);

}

int CommunicationInterface::leftSendPartialCommand(){

	uint8_t command[MCBCommand::partialCommandLength];
	read_lengths[LEFT_MOTOR] = lavy.getPartialCommand(command);
	return write(LEFT_MOTOR,command,MCBCommand::partialCommandLength);

}

int CommunicationInterface::rightSendPartialCommand(){

	uint8_t command[MCBCommand::partialCommandLength];
	read_lengths[RIGHT_MOTOR] = pravy.getPartialCommand(command);
	return write(RIGHT_MOTOR, command, MCBCommand::partialCommandLength);

}

int CommunicationInterface::leftSendControlCommand(){

	uint8_t command[MCBCommand::controlCommandLength];
	read_lengths[LEFT_MOTOR] = lavy.getControlCommand(command);
	return write(LEFT_MOTOR,command,MCBCommand::controlCommandLength);

}
int CommunicationInterface::rightSendControlCommand(){

	uint8_t command[MCBCommand::controlCommandLength];
	read_lengths[RIGHT_MOTOR] = pravy.getControlCommand(command);
	return write(RIGHT_MOTOR,command,MCBCommand::controlCommandLength);

}

int CommunicationInterface::writeMotors(){

    int succes = 0;
	switch(__builtin_ffs(getMotorControlBoardLeft()->getCommandID())){
		case REQUEST_COMMAND_FLAG:
            succes |= leftMCBStatusUpdate();
			break;
		case PARTIAL_COMMAND_FLAG-1:
            succes |= leftSendPartialCommand();
			//ROS_ERROR("partial");
			break;
		case CONTROL_COMMAND_FLAG:
            succes |= leftSendControlCommand();
			//ROS_ERROR("controll");
			break;
		default:
			ROS_ERROR("V Command ID laveho motora bola zla hodnota (toto by nikdy nemalo nastat)");
            succes = false;
	}
	switch(__builtin_ffs(getMotorControlBoardRight()->getCommandID())){
		case REQUEST_COMMAND_FLAG:
            succes |= rightMCBStatusUpdate();
			break;
		case PARTIAL_COMMAND_FLAG-1:
            succes |= rightSendPartialCommand();
			//ROS_ERROR("partial");
			break;
		case CONTROL_COMMAND_FLAG:
            succes |= rightSendControlCommand();
			//ROS_ERROR("controll");
			break;
		default:
			ROS_ERROR("V Command ID praveho motora bola zla hodnota (toto by nikdy nemalo nastat)");
            succes = false;
			}
	return succes;
}

CommunicationInterface::~CommunicationInterface(){

	close();
}

int CommunicationInterface::write(int id,uint8_t *dataWrite, int lengthWrite){

	if(active){
		int count;
		try{
			count = my_serials[id]->write(dataWrite, lengthWrite);
			my_serials[id]->flush();
			}catch (std::exception& e){
				ROS_ERROR("chyba pri zapise na port");
				active = false;
				close();
				return 0;
			}
		if (count != lengthWrite){
			ROS_ERROR("count!= lengthWrite. count = %d", count);
			return 0;
		}
		return 1 << id;
	}
	return 0;
}

bool CommunicationInterface::read(int id){

	if (active){
		uint8_t dataRead[read_lengths[id]];

		int count = -1;
		try{
			count = my_serials[id]->read(dataRead, read_lengths[id]);
		}catch (std::exception& e){
			ROS_ERROR("chyba pri citani z portu");
			active = false;
			close();
			return false;
		}

			if (count != read_lengths[id]){
				ROS_ERROR("count!= lengthRead. count = %d", count);
				return false;
			}

			if (dataRead[HEADER0] != HEADER || dataRead[HEADER1] != HEADER){
					//error
				ROS_ERROR("zla  hlavicka prislo \n");
				return false;
				}

				uint8_t device = dataRead[DEVICE_ADRESS];

			switch (dataRead[ANSWER_ID]){
				case MSG_OK:
					return true;
				case MSG_ERROR:
					ROS_ERROR("MSG ERROR %x", dataRead[ERR_CODE]);
					for(int i=0;i<count;i++)
						ROS_ERROR("%x ",dataRead[i]);
					return false;
				default:
					//TODO navratova hodnota + osetrenie
					convertMsg(&dataRead[ANSWER_ID], device);
				}
            new_data_status |= 1 < id;
			return true;

		}
		return false;

}

void CommunicationInterface::setFds(fd_set *set,int* flag){

	FD_ZERO(set);
	for(int i=0;i<my_serials.size();i++){
		if(!(*flag & (1<<i)))
			FD_SET(my_serials[i]->getFd(),set);
	}
}
int CommunicationInterface::getReadableFd(fd_set *set,int* flag){
	for(int i=0;i<3;i++){
		if(FD_ISSET(my_serials[i]->getFd(),set)){
			FD_CLR(my_serials[i]->getFd(),set);
			*flag |= 1 << i;
			return i;
		}
	}
	return -1;
}

void CommunicationInterface::setupPort(int sb,int p,int bs){

	/*nastavenie defaultnych parametrov*/
	//sb - stop bits
	//p  - parity
	//bs - byte size

	serial::stopbits_t stopBits;
	serial::parity_t parity;
	serial::bytesize_t byteSize;


	/*konvertovanie parametrov aby sa dali nastavit v serial class*/
	switch (sb)
	    {
		case 1: stopBits = serial::stopbits_one;
			break;
		case 2: stopBits = serial::stopbits_two;
			break;
		default: stopBits = serial::stopbits_one_point_five;
			 break;
		}
	switch (p)
	    {
		case 0: parity = serial::parity_none;
			break;
		case 1: parity = serial::parity_odd;
			break;
		case 2: parity = serial::parity_even;
			 break;
	}
	switch (bs)
	    {
		case 5: byteSize = serial::fivebits;
			break;
		case 6: byteSize = serial::sixbits;
			break;
		case 7: byteSize = serial::sevenbits;
			 break;
		case 8: byteSize = serial::eightbits;
			 break;
	}
	/*samotne setnutie parametrov*/
	BOOST_FOREACH(serial::Serial* serial,my_serials){
		serial->setStopbits(stopBits);
		serial->setParity(parity);
		serial->setBytesize(byteSize);
	}
}

