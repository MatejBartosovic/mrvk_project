//
// Created by matejko on 14.10.2018.
//

#ifndef PROJECT_SERIALCOMMUNICATIONINTERFACE_H
#define PROJECT_SERIALCOMMUNICATIONINTERFACE_H

#include <adis16488/ComunicationInterface.h>
#include <serial/serial.h>
namespace Adis16488{


class SerialCommunicationInterface : public Adis16488::ComunicationInterface{
public:
    SerialCommunicationInterface();
    virtual bool redRegisters(const std::vector<uint8_t> &adress, std::vector<uint16_t> &data) ;
    virtual bool writeRegisters(const std::vector<uint8_t> &adress, const std::vector<uint16_t> &data);
protected:
    serial::Serial serial;
};
}


#endif //PROJECT_SERIALCOMMUNICATIONINTERFACE_H
