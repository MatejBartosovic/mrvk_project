//
// Created by matejko on 23.3.2019.
//

#ifndef PROJECT_BUTTONEXCEPTION_H
#define PROJECT_BUTTONEXCEPTION_H

#include <exception>
#include <string>
namespace mrvk_gui{
    class ButtonsException : std::exception {
    public:
        ButtonsException(const std::string& str) : message(str) {

        }

        virtual const char* what() {
            return message.c_str();
        }

        std::string message;
    };
}
#define callService(serviceClient, msg)\
if(!serviceClient.call(msg)){ \
    throw ButtonsException("Block movement service is not running"); \
} \
if(!msg.response.success){ \
    throw ButtonsException("Service responded with error: " + msg.response.message); \
} \

#endif //PROJECT_BUTTONEXCEPTION_H
