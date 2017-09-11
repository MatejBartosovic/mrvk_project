#include "ros/ros.h"
#include <serial/serial.h>
#include <string>

#define NUM_DATA 14

struct dataADIStest{
    unsigned int counter = 0;
    float data[NUM_DATA];
};

int main(int argc, char **argv)
{
    dataADIStest readDataADIS;
    bool openFailed = false;
    //init read data
    for (int i = 0; NUM_DATA > i; i++)
    {
        readDataADIS.data[i] = 0;
    }
    ros::init(argc, argv, "testSerial");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    std::string port = "/dev/ttyUSB1";
    int baudRate = 9600;
    n.getParam("adis_16488_params/test_port_name", port);
    n.getParam("adis_16488_params/baud_rate", baudRate);

    serial::Serial* my_serial;
    try{
        my_serial = new serial::Serial(port, baudRate, serial::Timeout::simpleTimeout(100));

    }catch (std::exception& e){
        ROS_ERROR("ADIS16350: port don't open: %s", e.what());
        openFailed = true;
    }
    unsigned char startComChar = 0x42;
    unsigned char errorData = 0x43;

    if (!openFailed)
    {
        while(ros::ok())
        {
            //prepare data
            readDataADIS.counter = 151;
            for (int i = 0; i < NUM_DATA; i++)
            {
                readDataADIS.data[i] = i + 100;
            }

            int initCount = 0;
            while (initCount < 4)
            {
                my_serial->write((uint8_t *) (&startComChar), sizeof(unsigned char));
                initCount++;
            }
            my_serial->write((uint8_t *) (&readDataADIS), sizeof(dataADIStest));
            //my_serial->write((uint8_t *) (&readDataADIS), sizeof(dataADIStest));

            //write error data
            //my_serial->write((uint8_t *) (&errorData), sizeof(unsigned char));


            loop_rate.sleep();
        }
        my_serial->close();
    }

    return 0;
}

