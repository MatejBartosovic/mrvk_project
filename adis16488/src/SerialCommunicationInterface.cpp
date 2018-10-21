//
// Created by matejko on 14.10.2018.
//

#include "adis16488/SerialCommunicationInterface.h"
#include <adis16488/SerialMessage.h>
#include <iostream>
#include <algorithm>
namespace Adis16488 {
    SerialCommunicationInterface::SerialCommunicationInterface() : serial("/dev/ttyACM0",115200,serial::Timeout(10,10,0,10)){
    }
    bool SerialCommunicationInterface::redRegisters(const std::vector<uint8_t> &adress, std::vector<uint16_t> &data) {
        if(adress.size() > 255){
            throw ToManyRegistersToRead(adress.size());
        }
        Header header;
        header.start = SERIAL_HEADER_START_BYTE;
        header.size = adress.size();
        Footer footer;
        footer.end = SERIAL_FOOTER_END_BYTE;
        if(!((serial.write((uint8_t*)&header, sizeof(Header)) == sizeof(Header)) && (serial.write(adress) == header.size) && (serial.write((uint8_t*)&footer,sizeof(Footer)) == sizeof(Footer)))){
            std::cout << "Write failed" << std::endl;
            return false;
        }
        size_t toRead = header.size*2;
        data.resize(header.size);
        if(serial.read((uint8_t*)data.data(),toRead) != toRead){
            std::cout << "Read failed" << std::endl;
        }
        #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__ // data from adis are in big endian
        for (auto it = data.begin(); it != data.end(); ++it) {
            *it = __bswap_16(*it);
        }
        #endif
        for(int i = 0; i< data.size();i++){
            std::cout << data[i] << " ";
        }
        std::cout << std::endl;

    }
    bool SerialCommunicationInterface::writeRegisters(const std::vector<uint8_t> &adress, const std::vector<uint16_t> &data){

    }
}