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
    unsigned int counter = 0;
    float data[NUM_DATA];
};

struct packetADIS{
    unsigned char header[4];
    unsigned char numBytes = 0;
    uint32_t counter = 0;
    float data[NUM_DATA];
    unsigned char crc = 0;
    unsigned char footer[4];
}__attribute__((packed));

struct packetADISnohead{
    unsigned char numBytes = 0;
    uint32_t counter = 0;
    float data[NUM_DATA];
    //unsigned char crc = 0;
    unsigned char footer[4];
}__attribute__((packed));

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
    packetADIS readPacketADIS;
    packetADISnohead readPacketADISnohead;
    bool synched = false;
    bool openFailed = false;
    unsigned char dataSize;
    sensor_msgs::Imu imu;
    unsigned char calibrateCommand = 0;
    int retVal = 0;

    long long int dataCounter = 0;
    bool enable_print_data = false;
    //init read data
    for (int i = 0; NUM_DATA > i; i++)
    {
        readDataADIS.data[i] = 0;
        readPacketADIS.data[i] = 0;
    }
    ros::init(argc, argv, "comADIS");
    ros::NodeHandle n;
    std::string topicName = "imu_data";
    n.getParam("adis_16488_params/topic_name", topicName);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("chatter", 1);
    ros::ServiceServer calib_adis_srv_n = n.advertiseService("calib_adis_srv", calibAdis);
    ros::ServiceServer reset_adis_srv_n = n.advertiseService("reset_adis_srv", resetAdis);
    ros::ServiceServer reset_gyro_srv_n = n.advertiseService("reset_gyro_srv", resetGyro);

    std::string port = "/dev/ttyUSB0";
    int baudRate = 9600;
    n.getParam("adis_16488_params/port_name", port);
    n.getParam("adis_16488_params/baud_rate", baudRate);
    n.getParam("adis_16488_params/enable_print_data", enable_print_data);

    serial::Serial* my_serial;
    try{
        my_serial = new serial::Serial(port, baudRate, serial::Timeout::simpleTimeout(1000));

    }catch (std::exception& e){
        ROS_ERROR("ADIS16350: port don't open: %s", e.what());
        openFailed = true;
    }
    unsigned char startComChar;

    if (!openFailed)
    {
        while(ros::ok()) {
            if (!synched) {
                int initCount = 0;
                unsigned char headerConst;
                headerConst = 0x42;
                while (initCount < 4) {
                    ROS_ERROR("KOKOTI pojebany %d %d", initCount, headerConst);
                    my_serial->read((&startComChar), sizeof(unsigned char));
                    if (startComChar == headerConst) {
                        initCount++;
                        headerConst++;
                    } else {
                        ROS_ERROR("Error ADIS serial: unexpectet byte!");
                        initCount = 0;
                        headerConst = 0x42;
                    }
                }
                synched = true;
                printf("retVal: %d \n", my_serial->read((uint8_t *) (&readPacketADISnohead), sizeof(packetADISnohead)));
                printf("size: %d \n", sizeof(packetADISnohead));
                if (enable_print_data) {
                    printf("footer: %x", readPacketADISnohead.footer[0]);
                    printf(" %x", readPacketADISnohead.footer[1]);
                    printf(" %x", readPacketADISnohead.footer[2]);
                    printf(" %x", readPacketADISnohead.footer[3]);
                }
                ROS_ERROR("KOKOTsky size: %d ", readPacketADISnohead.numBytes);
                ROS_ERROR("Kokotsky counter: %d ", readPacketADISnohead.counter);
                for (int i = 0; i < NUM_DATA; i++)
                {
                    ROS_ERROR("DATA %d: %f", i, readPacketADISnohead.data[i]);
                }
            }
            printf("retVal: %d \n", my_serial->read((uint8_t *) (&readPacketADIS), sizeof(packetADIS)));
            printf("size %d \n", sizeof(packetADIS));
            if (enable_print_data) {
                printf("header: %x ", readPacketADIS.header[0]);
                printf(" %x ", readPacketADIS.header[1]);
                printf(" %x ", readPacketADIS.header[2]);
                printf(" %x ", readPacketADIS.header[3]);
                printf("\n");

                printf("footer: %x", readPacketADIS.footer[0]);
                printf(" %x", readPacketADIS.footer[1]);
                printf(" %x", readPacketADIS.footer[2]);
                printf(" %x", readPacketADIS.footer[3]);
            }

            if (readPacketADIS.header[0] != 0x42 ||
                readPacketADIS.header[0] != 0x43 ||
                readPacketADIS.header[0] != 0x44 ||
                readPacketADIS.header[0] != 0x45)
            {
                ROS_ERROR("Rozsynchronizovalo sa to!");
                synched = false;
            }

            //publish imu data
            dataCounter++;
            imu.header.frame_id = "imu";
            imu.header.seq = dataCounter;
            imu.header.stamp.nsec = ros::Time::now().nsec;
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
                    ROS_ERROR("DATA %d: %f", i, readPacketADIS.data[i]);
                }
            }
            std::cout << enable_print_data << std::endl;

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