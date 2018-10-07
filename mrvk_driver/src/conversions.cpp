#include <mrvk_driver/conversions.h>

Conversions::Conversions() : cameraPositionX(0),cameraPositionZ(0), inicialized(2,false),
							 posWheelsLast(2,0),posWheels(2,0), posActuators(2,0),posActuatorsLast(2,0) {
}

void Conversions::convertMsg(uint8_t *data, uint8_t device){
    boost::unique_lock<boost::mutex> lock(data_mutex);
	switch(device){
	case MAIN_BOARD_ADRESS:
		if (data[0] == 0x40 || data[0] == 0x02)
			answerMB_40_02(data);

		break;
	case LEFT_MOTOR_ADRESS:
		answerMCB(data, device);
		break;
	case RIGHT_MOTOR_ADRESS:
		answerMCB(data, device);
		break;
	}
}

bool Conversions::getStatusMotorErrors(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
    return statusMCB[0].i2c_communication_error && statusMCB[0].i2c_error_status && statusMCB[0].rs422_communication_error &&statusMCB[1].i2c_communication_error && statusMCB[1].i2c_error_status && statusMCB[1].rs422_communication_error;
}

bool Conversions::getPowerArm(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
    return statusMB.power_managment.arm_5V && statusMB.power_managment.arm_12V;
}

void Conversions::answerMB_40_02(uint8_t *data){

		statusMB.power_managment.MCBsSB_5V =  ((data[1] & 1) ? true : false);
		statusMB.power_managment.MCBs_12V =  ((data[1] & 2) ? true : false);
		statusMB.power_managment.videotransmitter = ((data[1] & 4) ? true : false);
		statusMB.power_managment.fan = ((data[1] & 8) ? true : false);
		statusMB.power_managment.laser =  ((data[1] & 16) ? true : false);
		statusMB.power_managment.gps =  ((data[1] & 32) ? true : false);
		statusMB.power_managment.arm_5V = ((data[1] & 64) ? true : false);
		statusMB.power_managment.arm_12V = ((data[1] & 128) ? true : false);
		statusMB.power_managment.pc2 = ((data[2] & 1) ? true : false);
		statusMB.power_managment.camera = ((data[2] & 2) ? true : false);

		statusMB.central_stop = ((data[3] & 1) ? true : false);
		statusMB.hardware_central_stop = ((data[3] & 2) ? true : false);
		statusMB.power_off_sequence = ((data[3] & 4) ? true : false);
		statusMB.full_battery = ((data[3] & 8) ? true : false);

		bool energy_source=((data[3] & 64) ? true : false);

		cameraPositionZ = (char2BToInt16(data[4], data[5]) - MBNULPOL1)/MBZISK1;
		cameraPositionX = (char2BToInt16(data[6], data[7]) - MBNULPOL2)/MBZISK2;
		//todo anal. hodnota 12V a 5V

		if (energy_source) //napajane z baterie
		{
			statusMB.battery.charge = char4BToUint32(data[8],data[9],data[10],data[11])/QBAT;
			float battery_voltage = (float)char2BToInt16(data[14],data[15])*MBB12;
			statusMB.battery.battery1_voltage = (float)char2BToInt16(data[16],data[17])*MBB1;
			statusMB.battery.battery2_voltage = battery_voltage - statusMB.battery.battery1_voltage;
			statusMB.battery.current = (float)char2BToInt16(data[18],data[19])*MBIB;

			if ((statusMB.battery.battery1_voltage < 13) || (statusMB.battery.battery2_voltage < 13)){
				//info.data=	"ROS_ERROR: napatie na baterii je pod 13V vypny podvozok!!!!!!!!!!!!!!!";
			 	//pubERROR.publish(info);
			}
		}
		else //napajanie zo zdroja
		{
			statusMB.battery.charge = -1;
			statusMB.battery.battery1_voltage = 0;
			statusMB.battery.battery2_voltage = 0;
			statusMB.battery.current = 0;
		}
		statusMB.temperature = (float)data[21]*MBT1;
}

void Conversions::answerMB_10(uint8_t *data){


		statusMB.central_stop = ((data[1] & 1) ? true : false);
		statusMB.hardware_central_stop = ((data[1] & 2) ? true : false);
		statusMB.power_off_sequence = ((data[1] & 4) ? true : false);
		statusMB.full_battery = ((data[1] & 8) ? true : false);

		bool energy_source=((data[1] & 64) ? true : false);

		//todo anal. hodnota 12V a 5V

		cameraPositionZ = (char2BToInt16(data[2], data[3]) - MBNULPOL1)/MBZISK1;
		cameraPositionX = (char2BToInt16(data[4], data[5]) - MBNULPOL2)/MBZISK2;


		if (energy_source) //napajane z baterie
		{
			statusMB.battery.charge = char4BToUint32(data[6],data[7],data[8],data[10])/QBAT;
			float battery_voltage = (float)char2BToInt16(data[12],data[13])*MBB12;
			statusMB.battery.battery1_voltage = (float)char2BToInt16(data[14],data[15])*MBB1;
			statusMB.battery.battery2_voltage = battery_voltage - statusMB.battery.battery1_voltage;
			statusMB.battery.current = (float)char2BToInt16(data[16],data[17])*MBIB;

			if ((statusMB.battery.battery1_voltage < 13) || (statusMB.battery.battery2_voltage < 13)){
				//info.data=	"ROS_ERROR: napatie na baterii je pod 13V vypny podvozok!!!!!!!!!!!!!!!";
			 	//pubERROR.publish(info);
			}
		}
		else //napajanie zo zdroja
		{
			statusMB.battery.charge = -1;
			statusMB.battery.battery1_voltage = 0;
			statusMB.battery.battery2_voltage = 0;
			statusMB.battery.current = 0;
		}
		statusMB.temperature = (float)data[19]*MBT1;
}


void Conversions::answerMCB(uint8_t *data, uint8_t device){
	int motorIndex = 0;

	if (device == LEFT_MOTOR_ADRESS){
		motorIndex = 0;
	}
	else motorIndex = 1;
		
		statusMCB[motorIndex].i2c_communication_error =  ((data[1] & 1) ? true : false);
		statusMCB[motorIndex].i2c_error_status =  ((data[1] & 2) ? true : false);
		statusMCB[motorIndex].rs422_communication_error = ((data[1] & 4) ? true : false);
		statusMCB[motorIndex].shifting = ((data[1] & 8) ? true : false);
		statusMCB[motorIndex].shifting_error =  ((data[1] & 16) ? true : false);

		velActuators[motorIndex] = char2BToInt16(data[2], data[3]);
		velWheels[motorIndex] = char2BToInt16(data[4], data[5]);

		posActuators[motorIndex] = char2BToInt16(data[6], data[7]);
		posWheels[motorIndex] = char2BToInt16(data[8], data[9]);

		if(!inicialized[motorIndex]){
			posWheelsLast[motorIndex] = posWheels[motorIndex];
			posActuatorsLast[motorIndex] = posActuators[motorIndex];
			inicialized[motorIndex] = true;
		}

		statusMCB[motorIndex].gear_position = data[10];

		statusMCB[motorIndex].gear_current = ((float)data[11])*MCBIGM;
	  	statusMCB[motorIndex].board_voltage = ((float)char2BToInt16(data[12],data[13]))*MCB5V;
	 	statusMCB[motorIndex].motor_voltage = ((float)char2BToInt16(data[14],data[15]))*MCBUM;
	  	statusMCB[motorIndex].motor_current = ((float)char2BToInt16(data[16],data[17]))*MCBIM;


}

mrvk_driver::Mb_status Conversions::getStatusMB(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	return statusMB;
}

mrvk_driver::Mcb_status Conversions::getStatusMCB(uint8_t device){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	if (device == LEFT_MOTOR_ADRESS)
		return statusMCB[0];
	else return statusMCB[1];
}

double Conversions::getVelLeftWheel(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	return velWheels[0] * ODOMETRY_CONSTANT_VELOCITY;
}

double Conversions::getVelRightWheel(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	return velWheels[1] * ODOMETRY_CONSTANT_VELOCITY;
}

double Conversions::getPosLeftWheel(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	int dif = (uint16_t)(posWheels[0] - posWheelsLast[0]);
	if (dif > 32767){
		dif = (uint16_t)(posWheelsLast[0] - posWheels[0]);
	}
	else
		dif *= -1;
	posWheelsLast[0] = posWheels[0];
	return dif * ODOMETRY_CONSTANT_POSITION;
}

double Conversions::getPosRightWheel(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	int dif = (uint16_t)(posWheels[1] - posWheelsLast[1]);
	if (dif > 32767){
		dif = (uint16_t)(posWheelsLast[1] - posWheels[1]);
	}
	else
		dif *= -1;
	posWheelsLast[1] = posWheels[1];
	return dif * ODOMETRY_CONSTANT_POSITION;
}

double Conversions::getVelLeftMotor(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	int dif = (uint16_t)(posActuators[0] - posActuatorsLast[0]);
	if (dif > 32767){
		dif = (uint16_t)(posActuatorsLast[0] - posActuators[0]);
	}
	else
		dif *= -1;
	posActuatorsLast[0] = posActuators[0];
	return dif * ODOMETRY_CONSTANT_POSITION;
}

double Conversions::getVelRightMotor(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	int dif = (uint16_t)(posActuators[1] - posActuatorsLast[1]);
	if (dif > 32767){
		dif = (uint16_t)(posActuatorsLast[1] - posActuators[1]);
	}
	else
		dif *= -1;
	posActuatorsLast[1] = posActuators[1];

	return dif * ODOMETRY_CONSTANT_POSITION;
}

double Conversions::getPosLeftMotor(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	return posActuators[0] * ODOMETRY_CONSTANT_POSITION;
}

double Conversions::getPosRightMotor(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
	return posActuators[1] * ODOMETRY_CONSTANT_POSITION;
}

double Conversions::getCameraPositionZ() {
	boost::unique_lock<boost::mutex> lock(data_mutex);
    return cameraPositionZ/180 *M_PI;
	}

double Conversions::getCameraPositionX(){
	boost::unique_lock<boost::mutex> lock(data_mutex);
    return cameraPositionX/180 *M_PI;
	}

int16_t Conversions::char2BToInt16(unsigned char H, unsigned char L)
{
	uint16_t result = (H << 8) | L;
	return (int16_t) result;
}

uint32_t Conversions::char4BToUint32(uint8_t HH, uint8_t H, uint8_t L, uint8_t LL){
	return (HH << 24) | (H << 16) | ( L<< 8) | LL;
}
