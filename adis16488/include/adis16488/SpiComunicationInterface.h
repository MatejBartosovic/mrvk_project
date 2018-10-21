//
// Created by matejko on 14.10.2018.
//

#ifndef PROJECT_SPICOMUNICATIONINTERFACE_H
#define PROJECT_SPICOMUNICATIONINTERFACE_H

#include <adis16488/ComunicationInterface.h>

namespace Adis16488{
    class SpiComunicationInterface : public Adis16488::ComunicationInterface {
    public:
        SpiComunicationInterface();
        virtual bool redRegisters(const std::vector<uint8_t> &adress, std::vector<uint16_t> &data);
        virtual bool writeRegisters(const std::vector<uint8_t> &adress, const std::vector<uint16_t> &data);
    };
}


#endif //PROJECT_SPICOMUNICATIONINTERFACE_H
