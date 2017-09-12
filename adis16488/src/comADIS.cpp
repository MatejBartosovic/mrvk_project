//myStruct newMyStruct;
//fileHandler.read((char *) (&newMyStruct), sizeof(myStruct));

#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include <std_srvs/Trigger.h>

#define NUM_DATA 14

#define VEL_X 0
#define VEL_Y 1
#define VEL_Z 2
#define VEL_X_DEL 3
#define VEL_Y_DEL 4
#define VEL_Z_DEL 5

#define ACCEL_X 6
#define ACCEL_Y 7
#define ACCEL_Z 8

#define MAG_X 9
#define MAG_Y 10
#define MAG_Z 11

#define ADIS_BARO 12
#define ADIS_TEMP 13

bool sendCommandCalib = false;
bool sendCommandResetAdis = false;
bool sendCommandResetGyro = false;

struct dataADIS{
    unsigned char header[4];
    unsigned char length = 0;
    unsigned int counter = 0;
    float data[NUM_DATA];
};

struct dataADISnohead{
    unsigned char length = 0;
    unsigned int counter = 0;
    float data[NUM_DATA];
};

bool calibAdis(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    sendCommandCalib = true;
    return true;
}
bool resetAdis(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    sendCommandResetAdis = true;
    return true;
}
bool resetGyro(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    sendCommandResetGyro = true;
    return true;
}

int main(int argc, char **argv)
{
    dataADIS readDataADIS;
    bool openFailed = false;
    unsigned char dataSize;
    sensor_msgs::Imu imu;
    unsigned char calibrateCommand = 0;

    long long int dataCounter = 0;
    bool enable_print_data = false;
    //init read data
    for (int i = 0; NUM_DATA > i; i++)
    {
        readDataADIS.data[i] = 0;
    }
    ros::init(argc, argv, "adis16488");
    ros::NodeHandle n("~");
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::ServiceServer calib_adis_srv_n = n.advertiseService("calib_adis_srv", calibAdis);
    ros::ServiceServer reset_adis_srv_n = n.advertiseService("reset_adis_srv", resetAdis);
    ros::ServiceServer reset_gyro_srv_n = n.advertiseService("reset_gyro_srv", resetGyro);

    std::string port = "/dev/ttyUSB0";
    int baudRate = 9600;
    n.getParam("port_name", port);
    n.getParam("baud_rate", baudRate);
    n.getParam("enable_print_data", enable_print_data);
    ROS_INFO("port %s",port.c_str());

    serial::Serial* my_serial;
    try{
        my_serial = new serial::Serial(port, baudRate, serial::Timeout::simpleTimeout(1000));

    }catch (std::exception& e){
        ROS_ERROR("ADIS16350: port don't open: %s", e.what());
        openFailed = true;
    }
    unsigned char startComChar;
    int readRet = 0;

    if (!openFailed)
    {
        while(ros::ok())
        {
            /*ROS_ERROR("KOKOT");
            int initCount = 0;
            while (initCount < 4)
            {
                ROS_ERROR("KOKOT2");
                readRet = my_serial->read((&startComChar), sizeof(unsigned char));
                ROS_ERROR("read ret %d %x", readRet,startComChar);
                if (readRet != 0)
                {
                    if (startComChar == 0x42)
                    {
                        initCount++;
                        ROS_ERROR("KOKOT3");
                    }
                    else
                    {
                        ROS_ERROR("Error ADIS serial: unexpectet byte!");
                        initCount = 0;
                    }
                }
                else
                {
                    ROS_ERROR("Nothing read!");
                }
            }
            my_serial->read((uint8_t *) (&dataSize), sizeof(unsigned char));*/
            readRet = my_serial->read((uint8_t *) (&readDataADIS), sizeof(dataADIS));

            ROS_ERROR("Read number: %d, header %x %x %x %x",readRet, readDataADIS.header[0], readDataADIS.header[1], readDataADIS.header[2], readDataADIS.header[3]);

            //publish imu data
            dataCounter++;
            imu.header.frame_id = "imu";
            imu.header.seq = dataCounter;
            imu.header.stamp = ros::Time::now();
            imu.angular_velocity.x = (double)readDataADIS.data[VEL_X];
            imu.angular_velocity.y = (double)readDataADIS.data[VEL_Y];
            imu.angular_velocity.z = (double)readDataADIS.data[VEL_Z];
            imu.linear_acceleration.x = (double)readDataADIS.data[ACCEL_X];
            imu.linear_acceleration.y = (double)readDataADIS.data[ACCEL_Y];
            imu.linear_acceleration.z = (double)readDataADIS.data[ACCEL_Z];
            imu.orientation = tf::createQuaternionMsgFromRollPitchYaw((double)readDataADIS.data[MAG_Z],(double)readDataADIS.data[MAG_Y],(double)readDataADIS.data[MAG_X]);

            //display read data
            if (enable_print_data)
            {
                ROS_ERROR("Data size: %d", dataSize);
                ROS_ERROR("Counter: %d", readDataADIS.counter);
                for (int i = 0; i < NUM_DATA; i++)
                {
                    ROS_ERROR("DATA %d: %f", i, readDataADIS.data[i]);
                }
            }

            imu_pub.publish(imu);
            ros::spinOnce();

            //send command to calibrate adis
            if (sendCommandCalib)
            {
                calibrateCommand = 50;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                calibrateCommand = 87;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                calibrateCommand = 14;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                sendCommandCalib = false;
            }
            //send command to reset adis
            if (sendCommandResetAdis)
            {
                calibrateCommand = 50;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                calibrateCommand = 87;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                calibrateCommand = 140;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                sendCommandResetAdis = false;
            }
            //send command to reset gyro
            if (sendCommandResetGyro)
            {
                calibrateCommand = 50;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                calibrateCommand = 87;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                calibrateCommand = 141;
                my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
                sendCommandResetGyro = false;
            }
        }
        my_serial->close();
    }

    return 0;
}