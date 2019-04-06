//
// Created by matejko on 5.4.2019.
//

#include <mrvk_driver/command.h>
#include <mrvk_driver/conversions.h>
#include <serial/serial.h>

int main(int argc, char *argv[]){
    MBCommand mbCommand(Conversions::MAIN_BOARD_ADRESS);

    mbCommand.setPC2(true);
    serial::Serial serialPort("/dev/MB",230400,serial::Timeout::simpleTimeout(500),serial::bytesize_t::eightbits,serial::parity_t::parity_odd);

    uint8_t dataToSend[MBCommand::controlCommandLength];
    int toRead = mbCommand.getUnitedCommand(dataToSend);
    serialPort.write(dataToSend,MBCommand::unitedCommandLength);

    uint8_t readData[toRead];
    int actualyRead = serialPort.read(readData,toRead);

    if (toRead != actualyRead){
       std::cout << "count!= lengthRead. count = " <<  actualyRead << std::endl;
        return 0;
    }

    if (readData[Conversions::HEADER0] != Conversions::HEADER || readData[Conversions::HEADER1] != Conversions::HEADER){
        //error
        std::cout << ("zla  hlavicka prislo") << std::endl;
        return 0;
    }

    switch (readData[Conversions::ANSWER_ID]) {
        case Conversions::MSG_OK:
            return 0;
        case Conversions::MSG_ERROR:
            std::cout << "MSG ERROR " << readData[Conversions::ERR_CODE] << std::endl;
            for (int i = 0; i < actualyRead; i++)
                ROS_ERROR("%x ", readData[i]);
            return 0;
        default:
            //TODO navratova hodnota + osetrenie
            std::cout << "Device config successful" << std::endl;
    }
    return 0;

}