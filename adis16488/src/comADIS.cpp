//myStruct newMyStruct;
//fileHandler.read((char *) (&newMyStruct), sizeof(myStruct));

#include "ros/ros.h"
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include <std_srvs/Trigger.h>
#include <signal.h>

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

#define DEG2RAD M_PI/180
#define GGG 9.798
//terminal gtkterm
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
    unsigned char crc = 0;
    unsigned char footer[4];
}__attribute__((packed));

serial::Serial* my_serial;

bool calibAdis(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    sendCommandCalib = true;
    int calibrateCommand = 50;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    calibrateCommand = 87;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    calibrateCommand = 139;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    sendCommandCalib = false;
    ROS_ERROR_STREAM("COMMAND NA KALIBRACIU POSLANY");
    sleep(30);
    ROS_ERROR_STREAM("GYRO NAKALIBROVANE mozete hybat robotom");
    res.success = true;
    return true;
}
bool resetAdis(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    sendCommandResetAdis = true;

    int calibrateCommand = 50;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    calibrateCommand = 87;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    calibrateCommand = 140;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    sendCommandResetAdis = false;
    res.success = true;
    return true;
}
bool resetGyro(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    sendCommandResetGyro = true;

    int calibrateCommand = 50;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    calibrateCommand = 87;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    calibrateCommand = 141;
    my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
    sendCommandResetGyro = false;
    res.success = true;

    return true;
}

void intHandler(int dummy) {
    ROS_ERROR("Closing port");
    my_serial->close();
}

int main(int argc, char **argv)
{


    signal(SIGINT, intHandler);


    dataADIS readDataADIS;
    packetADIS readPacketADIS;
    packetADISnohead readPacketADISnohead;
    bool synched = false;
    bool openFailed = false;
    bool only_z = true;
    unsigned char dataSize;
    sensor_msgs::Imu imu;
    unsigned char calibrateCommand = 0;
    int retVal = 0;
    uint8_t readData[70];

    long long int dataCounter = 0;
    bool enable_print_data = false;
    //init read data

    for (int i = 0; NUM_DATA > i; i++)
    {
        readDataADIS.data[i] = 0;
        readPacketADIS.data[i] = 0;
    }



    ros::init(argc, argv, "adis16488");
    ros::NodeHandle n("~");
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::ServiceServer calib_adis_srv_n = n.advertiseService("calib_adis_srv", calibAdis);
    ros::ServiceServer reset_adis_srv_n = n.advertiseService("reset_adis_srv", resetAdis);
    ros::ServiceServer reset_gyro_srv_n = n.advertiseService("reset_gyro_srv", resetGyro);

    std::string port = "/dev/ttyACM0";
    std::string imu_frame="imu";
    int baudRate = 115200;
    n.getParam("port_name", port);
    n.getParam("baud_rate", baudRate);
    n.getParam("enable_print_data", enable_print_data);
    n.getParam("only_z",only_z);
    n.getParam("imu_frame",imu_frame);

    ROS_INFO("port %s",port.c_str());

    try{
        my_serial = new serial::Serial(port, baudRate, serial::Timeout::simpleTimeout(100));

    }catch (std::exception& e){
        ROS_ERROR("ADIS16350: port don't open: %s", e.what());
        openFailed = true;
    }
    unsigned char startComChar;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    sleep(3);

    imu.linear_acceleration_covariance[0]=0.001;
    imu.linear_acceleration_covariance[1]=0;
    imu.linear_acceleration_covariance[2]=0;
    imu.linear_acceleration_covariance[3]=0;
    imu.linear_acceleration_covariance[4]=0.001;
    imu.linear_acceleration_covariance[5]=0;
    imu.linear_acceleration_covariance[6]=0;
    imu.linear_acceleration_covariance[7]=0;
    imu.linear_acceleration_covariance[8]=0.001;

    imu.angular_velocity_covariance[0]=0.0001;
    imu.angular_velocity_covariance[1]=0;
    imu.angular_velocity_covariance[2]=0;
    imu.angular_velocity_covariance[3]=0;
    imu.angular_velocity_covariance[4]=0.0001;
    imu.angular_velocity_covariance[5]=0;
    imu.angular_velocity_covariance[6]=0;
    imu.angular_velocity_covariance[7]=0;
    imu.angular_velocity_covariance[8]=0.0001;

    imu.angular_velocity_covariance[0]=0.0005;
    imu.angular_velocity_covariance[1]=0;
    imu.angular_velocity_covariance[2]=0;
    imu.angular_velocity_covariance[3]=0;
    imu.angular_velocity_covariance[4]=0.0005;
    imu.angular_velocity_covariance[5]=0;
    imu.angular_velocity_covariance[6]=0;
    imu.angular_velocity_covariance[7]=0;
    imu.angular_velocity_covariance[8]=0.0001;//0.0005;

    imu.orientation_covariance[0]=0.00001;
    imu.orientation_covariance[1]=0;
    imu.orientation_covariance[2]=0;
    imu.orientation_covariance[3]=0;
    imu.orientation_covariance[4]=0.00001;
    imu.orientation_covariance[5]=0;
    imu.orientation_covariance[6]=0;
    imu.orientation_covariance[7]=0;
    imu.orientation_covariance[8]=0.0001; //0.0005;


    if (!openFailed)
    {

        //reset adis
        /*int calibrateCommand = 50;
        my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
        calibrateCommand = 87;
        my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
        calibrateCommand = 140;
        my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
        sendCommandResetAdis = false;
        sleep(1);
        ROS_ERROR_STREAM("ORIENTACIE ZRESETOVANE");

        /*sleep(1);
        // int calibrateCommand = 50;
        my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
        calibrateCommand = 87;
        my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
        calibrateCommand = 139;
        my_serial->write((uint8_t *) (&calibrateCommand), sizeof(unsigned char));
        sendCommandCalib = false;

        sleep(30);
        ROS_ERROR_STREAM("GYRO NAKALIBROVANE Môžete hybat robotom");
        */
        while(ros::ok()) {

            if (!synched) {
                int initCount = 0;
                unsigned char headerConst = 0x42;
                ROS_WARN("ADIS serial: syncing");
                while (initCount < 4) {
                    //ROS_ERROR("KOKOTI pojebany %d %c", initCount, headerConst);
                    my_serial->read((&startComChar), sizeof(unsigned char));
                    //ROS_ERROR(" %c", startComChar);

                    if (startComChar == headerConst) {
                        initCount++;
                        headerConst++;
                    } else {
                        initCount = 0;
                        headerConst = 0x42;
                    }
                }
                ROS_WARN("ADIS serial: synced");
                synched = true;

                my_serial->read((uint8_t *) (&readPacketADISnohead), sizeof(packetADISnohead));
                //printf("retVal: %d \n", my_serial->read((uint8_t *) (&readPacketADISnohead), sizeof(packetADISnohead)));
                //printf("size: %d \n", sizeof(packetADISnohead));

                if (enable_print_data) {
                    printf("footer: %c", readPacketADISnohead.footer[0]);
                    printf(" %c", readPacketADISnohead.footer[1]);
                    printf(" %c", readPacketADISnohead.footer[2]);
                    printf(" %c", readPacketADISnohead.footer[3]);
                }

                //ROS_ERROR("KOKOTsky size: %d ", readPacketADISnohead.numBytes);
                //ROS_ERROR("Kokotsky counter: %d ", readPacketADISnohead.counter);
                /*for (int i = 0; i < NUM_DATA; i++)
                {
                    ROS_ERROR("DATA %d: %f", i, readPacketADISnohead.data[i]);
                }*/
            }

            int retval = my_serial->read((uint8_t *) (&readPacketADIS), sizeof(packetADIS));
            //printf("retVal: %d \n", retval);

            if(retval <=0 ){
                continue;
            }

            //printf("size %d \n", sizeof(packetADIS));

            if (enable_print_data) {
                printf("header: %c ", readPacketADIS.header[0]);
                printf(" %c ", readPacketADIS.header[1]);
                printf(" %c ", readPacketADIS.header[2]);
                printf(" %c ", readPacketADIS.header[3]);
                printf("\n");

                printf("footer: %c", readPacketADIS.footer[0]);
                printf(" %c", readPacketADIS.footer[1]);
                printf(" %c", readPacketADIS.footer[2]);
                printf(" %c \n", readPacketADIS.footer[3]);
            }

            if (readPacketADIS.header[0] != 0x42 ||
                readPacketADIS.header[1] != 0x43 ||
                readPacketADIS.header[2] != 0x44 ||
                readPacketADIS.header[3] != 0x45 ||
                readPacketADIS.footer[0] != 0x51 ||
                readPacketADIS.footer[1] != 0x52 ||
                readPacketADIS.footer[2] != 0x53 ||
                readPacketADIS.footer[3] != 0x54)
            {
                ROS_ERROR("Rozsynchronizovala sa komunikacia adis16488!");
                synched = false;
            }

            //publish imu data
            dataCounter++;

            if(only_z){

                imu.header.frame_id = imu_frame;
                imu.header.seq = dataCounter;
                imu.header.stamp = ros::Time::now();
                imu.angular_velocity.x = 0;
                imu.angular_velocity.y = 0;
                imu.angular_velocity.z = (double)readPacketADIS.data[VEL_Z]*DEG2RAD;
                imu.linear_acceleration.x = (double)readPacketADIS.data[ACCEL_X]*GGG;
                imu.linear_acceleration.y = (double)readPacketADIS.data[ACCEL_Y]*GGG;
                imu.linear_acceleration.z = (double)readPacketADIS.data[ACCEL_Z]*GGG;
		//double gyro_int_z = (double)readPacketADIS.data[VEL_Z_DEL]*DEG2RAD;
		//ROS_WARN("rot z = %f", gyro_int_z);
                imu.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(double)readPacketADIS.data[VEL_Z_DEL]*DEG2RAD);

            }
            else{
                imu.header.frame_id = imu_frame;
                imu.header.seq = dataCounter;
                imu.header.stamp = ros::Time::now();
                imu.angular_velocity.x = (double)readPacketADIS.data[VEL_X]*DEG2RAD;
                imu.angular_velocity.y = (double)readPacketADIS.data[VEL_Y]*DEG2RAD;
                imu.angular_velocity.z = (double)readPacketADIS.data[VEL_Z]*DEG2RAD;
                imu.linear_acceleration.x = (double)readPacketADIS.data[ACCEL_X]*GGG;
                imu.linear_acceleration.y = (double)readPacketADIS.data[ACCEL_Y]*GGG;
                imu.linear_acceleration.z = (double)readPacketADIS.data[ACCEL_Z]*GGG;
                imu.orientation = tf::createQuaternionMsgFromRollPitchYaw((double)readPacketADIS.data[VEL_X_DEL]*DEG2RAD,(double)readPacketADIS.data[VEL_Y_DEL]*DEG2RAD,(double)readPacketADIS.data[VEL_Z_DEL]*DEG2RAD);
            }

            //display read data
            if (enable_print_data)
            {
                ROS_ERROR("Data size: %d", readPacketADIS.numBytes);
                ROS_ERROR("Counter: %d", readPacketADIS.counter);
                for (int i = 0; i < NUM_DATA; i++)
                {
                    ROS_ERROR("DATA %d: %f", i, readPacketADIS.data[i]);
                }
            }
            //std::cout << enable_print_data << std::endl;

            imu_pub.publish(imu);
        }
        my_serial->close();
    }

    return 0;
}
