//
// Created by matejko on 14.10.2018.
//

#ifndef PROJECT_COMUNICATIONINTERFACE_H
#define PROJECT_COMUNICATIONINTERFACE_H

#include <stdint.h>
#include <vector>
#include <string>

namespace Adis16488{
    class ComunicationInterface {
    public:
        ComunicationInterface(){

        }
        virtual bool redRegisters(const std::vector<uint8_t> &adress, std::vector<uint16_t> &data) = 0;
        virtual bool writeRegisters(const std::vector<uint8_t> &adress, const std::vector<uint16_t> &data) = 0;
    };
class ComunicationInterfaceException : public std::exception {
    public:
        ComunicationInterfaceException(std::string s) : s(s)  {

        }
        virtual const char* what() const throw() override {
            return s.c_str();
        }

    private:
        std::string s;
};
class ToManyRegistersToRead : public ComunicationInterfaceException{
public:
    ToManyRegistersToRead(size_t size) : ComunicationInterfaceException("To many Registers to read: " + std::to_string(size)){

    }
};
}


#endif //PROJECT_COMUNICATIONINTERFACE_H
