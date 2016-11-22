#include <mrvk_driver/communication_interface.h>
#include <boost/foreach.hpp>

CommunicationInterface::CommunicationInterface(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize):
mb(MAIN_BOARD_ADRESS), pravy(RIGHT_MOTOR_ADRESS), lavy(LEFT_MOTOR_ADRESS){

	pthread_mutex_init(&callbacks_mutex,NULL);
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

	ROS_ERROR("som tu1");
	std::vector<int> fds;
	BOOST_FOREACH(serial::Serial* serial,my_serials){
		ROS_ERROR("som tu2");

		if (serial->isOpen())
			ROS_ERROR("port otvoreny"); //TODO prerobit na info
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
	ROS_ERROR("0 = %d, 0 = %d, 0 = %d max = %d",fds[0],fds[1],fds[2],fd_max);
	ROS_ERROR("init hotovy"); //TODO prerobit na info

		return true;
}

bool CommunicationInterface::isActive(){
	return active;
}

void CommunicationInterface::setMotorsVel(double left_vel,double right_vel){
	lavy.setMotorSpeed(left_vel);
	pravy.setMotorSpeed(-right_vel);

}

//TODO pojde prec
bool CommunicationInterface::setSpeedLeftMotor(double speed){

	uint8_t command[MCBCommand::partialCommandLength];
	lavy.setMotorSpeed(speed);
	read_lengths[LEFT_MOTOR] = lavy.getPartialCommand(command);
	write(LEFT_MOTOR,command,MCBCommand::partialCommandLength);

}

//TODO pojde prec
bool CommunicationInterface::setSpeedRightMotor(double speed){

	uint8_t command[MCBCommand::partialCommandLength];
	pravy.setMotorSpeed(speed);
	read_lengths[RIGHT_MOTOR] = pravy.getPartialCommand(command);
	write(RIGHT_MOTOR,command,MCBCommand::partialCommandLength);

}

/*bool CommunicationInterface::resetCentralStop(){

		if (!active)
			return false;

		 uint8_t MCB_command[21];
		 uint8_t MB_command[21];
		 uint8_t request[5];

		 mb.setCentralStop(false);
		 int mb_read_length = mb.getControlCommand(MB_command);
		 int request_read_length = mb.getRequestCommand(request);

		 uint8_t answer[request_read_length];


		int mcb_read_length;
		 my_serial->flush();
			ROS_ERROR("commands are generated and port is flushed");

			mb_mutex.lock();
		for (int i = 0; i<3;i++){

			lavy.setErrFlags(true,true); // nulovanie flagov, po zapise budu opat 0
			mcb_read_length = lavy.getControlCommand(MCB_command);

			if (!writeAndRead(MCB_command, MCBCommand::controlCommandLength, mcb_read_length, 1)){

				ROS_ERROR("central stop error in reset flags l - write");
				mb_mutex.unlock();
				return false;
			}
			ROS_ERROR("flags in left motor are reset");


			pravy.setErrFlags(true,true); // nulovanie flagov, po zapise budu opat 0
			pravy.getControlCommand(MCB_command);

			if (!writeAndRead(MCB_command, MCBCommand::controlCommandLength, mcb_read_length, 1)){


						ROS_ERROR("central stop error in reset flags r - write");
						mb_mutex.unlock();
						return false;
					}
			ROS_ERROR("flags in right motor are reset");


			usleep(100000);


			if (!writeAndRead(MB_command, MBCommand::controlCommandLength, mb_read_length, 1)){

						ROS_ERROR("central stop error in write");
						mb_mutex.unlock();
						return false;
					}
			ROS_ERROR("command reset central stop is correctly send");

		 	for (int j = 0; j < 3; j++)
		 	{
		 		usleep(400000);

		 		write_mutex.lock();
		 		if (!tryWrite(request, Command::requestCommandLength, 0)){
		 					//reset_central_stop_mutex.unlock();
		 					//write_mutex.unlock();
							ROS_ERROR("central stop error in request - write");
							write_mutex.unlock();
		 					mb_mutex.unlock();
							return false;
		 				}
				ROS_ERROR("request command are send");

		 		int result_byte = my_serial->read(answer, request_read_length);

		 		if (!(answer[7] & 1))
		 		{
		 			ROS_ERROR("unlock");
					 write_mutex.unlock();
					 mb_mutex.unlock();
					 return true; //central stop odblokovany 0 = okej, 1 = central stop in progress
		 		}

		 		write_mutex.unlock();
		 		ROS_ERROR("central stop is still not reset");
		 	}

		 }
		mb_mutex.unlock();

		 return false;
}*/

//todo pravdepodobne sa asi tiez bude mazat
bool CommunicationInterface::resetFlags(){

	uint8_t command[MCBCommand::controlCommandLength];

		lavy.setErrFlags(true,true);		//todo dorobit aj nastavenie regulatora kamery
		lavy.setMotorSpeed(0);
		lavy.setGearPosition(0);
		int read_length = lavy.getControlCommand(command); // dlzka, ktora je ocakavana na reade

		if ( !write(LEFT_MOTOR, command, MCBCommand::controlCommandLength))
			return false;

		read(read_length);

		pravy.setErrFlags(true,true);
		pravy.setMotorSpeed(0);
		pravy.setGearPosition(0);
		pravy.getControlCommand(command);

		if ( !write(RIGHT_MOTOR, command, MCBCommand::controlCommandLength))
			return false;

		return read(read_length);
}

//todo dorobit kameru rychlost otacania
bool CommunicationInterface::setCameraVelocity(double linearX, double angularZ){

	uint8_t command[MBCommand::controlCommandLength];

	static double oldLinearX = 0;
	static double oldAngularZ = 0;

	if ((oldLinearX == linearX) && (oldAngularZ == angularZ)) //ak pride rovnaka sprava ako pred tym, callback sa moze ukoncit
		return true;

	oldLinearX = linearX;
	oldAngularZ = angularZ;

	mb.setPosRotCam(false);
 	mb.setKameraVelocity(angularZ,linearX);
 	int read_length = mb.getControlCommand(command);

	 //return writeAndRead(command, MBCommand::controlCommandLength, read_length, 1);

}
//todo dorobit kameru - poloha
bool CommunicationInterface::setCameraPosition(double linearX, double angularZ){

	uint8_t command[MBCommand::controlCommandLength];

		static double oldLinearX = 0;
		static double oldAngularZ = 0;

		if ((oldLinearX == linearX) && (oldAngularZ == angularZ)) //ak pride rovnaka sprava ako pred tym, callback sa moze ukoncit
			return true;

		oldLinearX = linearX;
		oldAngularZ = angularZ;

		mb.setPosRotCam(true);
	 	mb.setKameraVelocity(angularZ,linearX);
	 	int read_length = mb.getControlCommand(command);

	// return writeAndRead(command, MBCommand::controlCommandLength, read_length, 1);

}


/*
 * volat iba pred zacatkom komunikacneho cyklu!!!!!!
 * jednorazovy prikaz pre inicializacne nastavenia
*/

bool CommunicationInterface::setMainBoard(SET_MAIN_BOARD *param){
	//TODO cekni toto miso prerobil somm to
	/*uint8_t command[MBCommand::controlCommandLength];
	mb.setParamatersMB(param);
	int read_length = mb.getControlCommand(command);

	if(! write(my_serials[MAIN_BOARD], command, MBCommand::controlCommandLength))
		return false;
	else return read(MAIN_BOARD);*/
	mb.setParamatersMB(param);
	
}

/*
 * volat iba pred zacatkom komunikacneho cyklu!!!!!!
 * jednorazovy prikaz pre inicializacne nastavenia
 */
bool CommunicationInterface::setMotorParameters(REGULATOR_MOTOR regulator, bool regulation_type){

	/*uint8_t command[MCBCommand::controlCommandLength];

	lavy.setRegulatorPID(regulator);
	lavy.setMotorControl(regulation_type);
	int read_length = lavy.getControlCommand(command);

	if ( !write(LEFT_MOTOR, command, MCBCommand::controlCommandLength))
		return false;

	read(read_length);

	pravy.setRegulatorPID(regulator);
	pravy.setMotorControl(regulation_type);
	pravy.getControlCommand(command);

	if ( !write(RIGHT_MOTOR, command, MCBCommand::controlCommandLength))
		return false;

	return read(read_length);*/
	lavy.setRegulatorPID(regulator);
	lavy.setMotorControl(regulation_type);
	pravy.setRegulatorPID(regulator);
	pravy.setMotorControl(regulation_type);

}

bool CommunicationInterface::MBStatusUpdate(){

	uint8_t command[MBCommand::requestCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getRequestCommand(command);
	return write(MAIN_BOARD, command, MBCommand::requestCommandLength);

}

bool CommunicationInterface::MBSendPartialCommd(){

	uint8_t command[MBCommand::partialCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getPartialCommand(command);
	processCallbacks(MAIN_BOARD);
	return write(MAIN_BOARD, command, MBCommand::partialCommandLength);
}

bool CommunicationInterface::MBSendControlCommd(){

	uint8_t command[MBCommand::controlCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getControlCommand(command);
	processCallbacks(MAIN_BOARD);
	return write(MAIN_BOARD, command, MBCommand::controlCommandLength);
}

bool CommunicationInterface::MBSendUnitedCommd(){

	uint8_t command[MBCommand::unitedCommandLength];
	read_lengths[MAIN_BOARD] =  mb.getUnitedCommand(command);
	processCallbacks(MAIN_BOARD);
	return write(MAIN_BOARD, command, MBCommand::unitedCommandLength);
}


bool CommunicationInterface::leftMCBStatusUpdate(){

	uint8_t command[MCBCommand::requestCommandLength];
	read_lengths[LEFT_MOTOR] = lavy.getRequestCommand(command);
	return write(LEFT_MOTOR,command,MCBCommand::requestCommandLength);

}

bool CommunicationInterface::rightMCBStatusUpdate(){

	uint8_t command[MCBCommand::requestCommandLength];
	read_lengths[RIGHT_MOTOR] = pravy.getRequestCommand(command);
	return write(RIGHT_MOTOR, command, MCBCommand::requestCommandLength);

}

bool CommunicationInterface::leftSendPartialCommand(){

	uint8_t command[MCBCommand::partialCommandLength];
	read_lengths[LEFT_MOTOR] = lavy.getPartialCommand(command);
	processCallbacks(LEFT_MOTOR);
	return write(LEFT_MOTOR,command,MCBCommand::partialCommandLength);

}

bool CommunicationInterface::rightSendPartialCommand(){

	uint8_t command[MCBCommand::partialCommandLength];
	read_lengths[RIGHT_MOTOR] = pravy.getPartialCommand(command);
	processCallbacks(RIGHT_MOTOR);
	return write(RIGHT_MOTOR, command, MCBCommand::partialCommandLength);

}

bool CommunicationInterface::leftSendControlCommand(){

	uint8_t command[MCBCommand::controlCommandLength];
	read_lengths[LEFT_MOTOR] = lavy.getControlCommand(command);
	processCallbacks(LEFT_MOTOR);
	return write(LEFT_MOTOR,command,MCBCommand::controlCommandLength);

}
bool CommunicationInterface::rightSendControlCommand(){

	uint8_t command[MCBCommand::controlCommandLength];
	read_lengths[RIGHT_MOTOR] = pravy.getControlCommand(command);
	processCallbacks(RIGHT_MOTOR);
	return write(RIGHT_MOTOR,command,MCBCommand::controlCommandLength);

}

//TODO pojde prec
bool CommunicationInterface::sendMainBoardStruct(){

	 //TODO cez switch ten je asi rychlejsi ale neviem a malocc
	 int commandID =mb.getCommandID();
	 ROS_ERROR("mb command ID %d", commandID);
	if (commandID & UNITED_COMMAND_FLAG){

		uint8_t command[MBCommand::unitedCommandLength];
		processCallbacks(MAIN_BOARD);
		read_lengths[MAIN_BOARD] =  mb.getUnitedCommand(command); // dlzka, ktora je ocakavana na reade
		write(MAIN_BOARD, command, MBCommand::unitedCommandLength);
	}

	else if (commandID & CONTROL_COMMAND_FLAG){
			uint8_t command[MBCommand::controlCommandLength];
			processCallbacks(MAIN_BOARD);
			read_lengths[MAIN_BOARD] =  mb.getControlCommand(command); // dlzka, ktora je ocakavana na reade
			write(MAIN_BOARD, command, MBCommand::controlCommandLength);
		}

	else if (commandID & PARTIAL_COMMAND_FLAG){
		uint8_t command[MBCommand::partialCommandLength];
		processCallbacks(MAIN_BOARD);
		read_lengths[MAIN_BOARD] =  mb.getPartialCommand(command); // dlzka, ktora je ocakavana na reade
		write(MAIN_BOARD, command, MBCommand::partialCommandLength);
	}

	return false;
}

CommunicationInterface::~CommunicationInterface(){

	close();
}

void CommunicationInterface::close(){
	if (active){
		BOOST_FOREACH(serial::Serial* serial,my_serials){
			serial->close();
		}
		active = false;
	}
}


bool CommunicationInterface::write(int id,uint8_t *dataWrite, int lengthWrite){
	if(active){
		int count = -1;
		try{
			count = my_serials[id]->write(dataWrite, lengthWrite);
			my_serials[id]->flush();
			}catch (std::exception& e){
				ROS_ERROR("chyba pri zapise na port");
				active = false;
				close();
				notifyAllCallbacks(false,id);
				return false;
			}
		if (count != lengthWrite){
			ROS_ERROR("count!= lengthWrite. count = %d", count);
			notifyAllCallbacks(false,id);
			return false;
		}
		return true;
	}
	return false;
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
			notifyAllCallbacks(false,id);
			return false;
		}

			if (count != read_lengths[id]){
				ROS_ERROR("count!= lengthRead. count = %d", count);
				notifyAllCallbacks(false,id);
				return false;
			}

			if (dataRead[HEADER0] != HEADER || dataRead[HEADER1] != HEADER){
					//error
				ROS_ERROR("zla  hlavicka prislo \n");
				notifyAllCallbacks(false,id);
				return false;
				}

				uint8_t device = dataRead[DEVICE_ADRESS];

			switch (dataRead[ANSWER_ID]){
				case MSG_OK:
					notifyAllCallbacks(true,id);
					return true;
				case MSG_ERROR:
					ROS_ERROR("MSG ERROR %x", dataRead[ERR_CODE]);
					for(int i=0;i<count;i++)
						ROS_ERROR("%x ",dataRead[i]);
					notifyAllCallbacks(false,id);
					return false;
				default:
					//TODO navratova hodnota + osetrenie
					convertMsg(&dataRead[ANSWER_ID], device);
				}
			notifyAllCallbacks(true,id);
			return true;

		}
		notifyAllCallbacks(false,id);
		return false;

}

void CommunicationInterface::processCallbacks(int id){
	pthread_mutex_lock(&callbacks_mutex);
	processed_callbacks[id] = waitings_callbacks[id];
	waitings_callbacks[id].clear();
	pthread_mutex_unlock(&callbacks_mutex);
}

void CommunicationInterface::registerMutex(pthread_mutex_t* mutex,int id){
	pthread_mutex_lock(&callbacks_mutex);
	waitings_callbacks[id].push_back(mutex);
	pthread_mutex_unlock(&callbacks_mutex);
}

void CommunicationInterface::notifyAllCallbacks(bool response,int id){
	pthread_mutex_lock(&callbacks_mutex);
	callback_res[id] = response;
	BOOST_FOREACH(pthread_mutex_t* callback,processed_callbacks[id]){
		pthread_mutex_unlock(callback);
	}
	pthread_mutex_unlock(&callbacks_mutex);
}
bool CommunicationInterface::getCallbackResult(int id){
	bool res;
	pthread_mutex_lock(&callbacks_mutex);
	res = callback_res[id];
	pthread_mutex_unlock(&callbacks_mutex);
	return res;
}

int CommunicationInterface::waitToRead(){
		//my_serials[0]->flush();
	fd_set read_set;
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 500000; //500ms
	int readable_fds;
	int sucesfull_readings = 0;
	int index;
	int setFlags = 0;
	while(1){
		setFds(&read_set,&setFlags);
		//select aktualizuje timeout a aktualizuje set
		//ROS_ERROR("mb %d mcbl %d mcbr %d",(int)FD_ISSET(my_serials[0]->getFd(),&read_set),(int)FD_ISSET(my_serials[1]->getFd(),&read_set),(int)FD_ISSET(my_serials[2]->getFd(),&read_set));
		readable_fds = select(fd_max+1,&read_set,NULL,NULL,&timeout);
		//ROS_ERROR("za selectom readable fds = %d timeout = %d available bytes = %d",readable_fds,(int)timeout.tv_usec,(int)my_serials[0]->available());
		if (readable_fds == -1){
			ROS_ERROR("select failed");
			if(timeout.tv_usec == 0)
				return sucesfull_readings;
			else
				continue; //TODO nieco lebsie tu vymisliet
		}
		for(int i = 0;i<readable_fds;i++){
			index = getReadableFd(&read_set,&setFlags); // vrati file descriptor ktori je citatelny
			if(index !=-1){
				sucesfull_readings ++;
				read(index);
			}
		}
		if(timeout.tv_usec == 0 || sucesfull_readings ==my_serials.size())
			break;
	}
	return sucesfull_readings;
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

MBCommand* CommunicationInterface::getMainBoard(){
	return &mb;
}

MCBCommand* CommunicationInterface::getMotorControlBoardLeft(){
	return &lavy;
}
MCBCommand* CommunicationInterface::getMotorControlBoardRight(){
	return &pravy;
}

//todo nepojde prec?
bool CommunicationInterface::isAnswerOk(){
	//todo znovu prerobena funkcia iba na mb
	//TODO preco je velkost 7 a citas 6? Miso - neviem mozno preklep. Tu ale treba domysliet problem, ze ked pride odpoved OK tak pride 6 bytov ale ked pride ERROR tak 7 bytov
	uint8_t data[7];
	int result_byte = my_serials[0]->read(data, 6);

	if (data[4] == MSG_OK)
		return true;
	else return false;

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
