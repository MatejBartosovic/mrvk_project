#ifndef COMMUNICATION_INTERFACE_H_
#define COMMUNICATION_INTERFACE_H_

#include <serial/serial.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <kv01_driver/communication_interface.h>

namespace Kv01 {

    class CommunicationInterface {
    public:

        CommunicationInterface(std::vector<std::string> ports, int baudrate, int stopBits, int parity, int byteSize);
        ~CommunicationInterface(){}

        bool init();
        bool isActive();
        void close();

        void readStates(double &joint);

    private:

        void setupPort(int sb,int p,int bs);

        std::vector<serial::Serial *> my_serials;
        bool active;
        bool blocked;
        std::vector<std::string> ports;
        int fd_max;
        std::vector<int> read_lengths;
        int baud;
        int byteSize;
        int stopBits;
        int parity;
    };
}


#endif /* COMMUNICATION_INTERFACE_H_ */

