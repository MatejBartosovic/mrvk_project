//
// Created by matejko on 5.4.2019.
//

#include <mrvk_driver/command.h>
#include <mrvk_driver/conversions.h>
#include <serial/serial.h>
#include <iostream>
#include <fstream>
#include <syslog.h>

int main(int argc, char *argv[]){
    //setlogmask(LOG_UPTO(LOG_NOTICE));
std::string port;
    openlog("mrvk init", 0, LOG_USER);

    if(argc > 1){
	port = "/dev/" + std::string(argv[1]);
    }
    else{
    	port = "/dev/MB";
    }
    syslog(LOG_INFO,"Configuring mrvk main board on device %s",port.c_str());
    MBCommand mbCommand(Conversions::MAIN_BOARD_ADRESS);

    mbCommand.setPC2(true);
    serial::Serial serialPort(port,230400,serial::Timeout::simpleTimeout(500),serial::bytesize_t::eightbits,serial::parity_t::parity_odd);

    uint8_t dataToSend[MBCommand::controlCommandLength];
    int toRead = mbCommand.getUnitedCommand(dataToSend);
    serialPort.write(dataToSend,MBCommand::unitedCommandLength);

    uint8_t readData[toRead];
    int actualyRead = serialPort.read(readData,toRead);

    if (toRead != actualyRead){
	syslog(LOG_ERR,"count!= lengthRead. count = %d",actualyRead);
	closelog();
        return 0;
    }

    if (readData[Conversions::HEADER0] != Conversions::HEADER || readData[Conversions::HEADER1] != Conversions::HEADER){
        //error
	syslog(LOG_ERR,"Header mismatch, exoected: %x, received: %x",Conversions::HEADER,readData[Conversions::HEADER0]);
	closelog();
        return 0;
    }

    switch (readData[Conversions::ANSWER_ID]) {
        case Conversions::MSG_OK:
	    closelog();		
            return 0;
        case Conversions::MSG_ERROR:
	    syslog(LOG_ERR,"Error msg received %x",readData[Conversions::ERR_CODE]);
            for (int i = 0; i < actualyRead; i++)
                ROS_ERROR("%x ", readData[i]);
	    closelog();
            return 0;
        default:
            //TODO navratova hodnota + osetrenie
	    syslog(LOG_INFO,"Device config successful");
    }
   closelog(); 
   return 0;
}
