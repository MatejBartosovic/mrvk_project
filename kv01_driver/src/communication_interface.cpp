#include <kv01_driver/communication_interface.h>
#include <kv01_driver/command.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>

/*
 *
 *
 * Public functions
 *
 *
 * */
namespace Kv01 {

    CommunicationInterface::CommunicationInterface(std::vector <std::string> ports, int baudrate, int stopBits,
                                                   int parity, int byteSize)
    {

        if (ports.size() != 1)
            throw std::invalid_argument("Invalid serial ports.");

        //read_lengths.resize(3);
        this->ports = ports;
        this->baud = baudrate;
        this->stopBits = stopBits;
        this->byteSize = byteSize;
        this->parity = parity;
        active = false;
    }

    bool CommunicationInterface::init() {


        if (active)        //ak vsetko bezi v poriadku nemusi sa znova volat init
            return true;
        try {
            BOOST_FOREACH(std::string port, ports) {
                my_serials.push_back(new serial::Serial(port, baud, serial::Timeout::simpleTimeout(50)));
            }
        } catch (std::exception &e) {
            ROS_ERROR("port sa neotvoril ale pokracujem aj tak treba fixnut");
            active = true;
            return true;
        }

        std::vector<int> fds;
        BOOST_FOREACH(serial::Serial * serial, my_serials)
        {

            if (serial->isOpen())
                ROS_INFO("port otvoreny");
            else {
                ROS_ERROR("port sa neotvoril");
                active = false;
                return false;
            }
            try {
                setupPort(stopBits, parity, byteSize);
                serial->flush();

            } catch (std::exception &e) {
                ROS_ERROR("nenastavili sa parametre portu");
                active = false;
                return false;
            }
            fds.push_back(serial->getFd());
        }
        fd_max = -1;
        BOOST_FOREACH(int fd, fds){
            if (fd > fd_max)
                fd_max = fd;
        }
        active = true;
        ROS_INFO("init done");

        return true;
    }

    bool CommunicationInterface::isActive() {
        return active;
    }

    void CommunicationInterface::close() {
        if (active) {
            BOOST_FOREACH(serial::Serial * serial, my_serials)
            {
                serial->close();
            }
            active = false;
        }
    }

    void CommunicationInterface::readStates(double &joint){

        /*common_functions.ArmCreateD3(message,joint);

        pthread_mutex_lock( &comm_lock_mutex);
            if (!serial_comm(message,rBuffer))
            {
                ROS_ERROR("Unknow command alebo Timeout pre citanie pozicie");
                pthread_mutex_unlock( &comm_lock_mutex);
            }
        pthread_mutex_unlock( &comm_lock_mutex);
        joint=common_functions->ArmReceiveD3(rBuffer,joint);
        return;*/
    }

    void CommunicationInterface::setupPort(int sb, int p, int bs) {

        /*nastavenie defaultnych parametrov*/
        //sb - stop bits
        //p  - parity
        //bs - byte size

        serial::stopbits_t stopBits;
        serial::parity_t parity;
        serial::bytesize_t byteSize;


        /*konvertovanie parametrov aby sa dali nastavit v serial class*/
        switch (sb) {
            case 1:
                stopBits = serial::stopbits_one;
                break;
            case 2:
                stopBits = serial::stopbits_two;
                break;
            default:
                stopBits = serial::stopbits_one_point_five;
                break;
        }
        switch (p) {
            case 0:
                parity = serial::parity_none;
                break;
            case 1:
                parity = serial::parity_odd;
                break;
            case 2:
                parity = serial::parity_even;
                break;
        }
        switch (bs) {
            case 5:
                byteSize = serial::fivebits;
                break;
            case 6:
                byteSize = serial::sixbits;
                break;
            case 7:
                byteSize = serial::sevenbits;
                break;
            case 8:
                byteSize = serial::eightbits;
                break;
        }
        /*samotne setnutie parametrov*/
        BOOST_FOREACH(serial::Serial * serial, my_serials)
        {
            serial->setStopbits(stopBits);
            serial->setParity(parity);
            serial->setBytesize(byteSize);
        }
    }
}

