//
// Created by root on 4/4/17.
//

#ifndef PROJECT_COMMAND_H
#define PROJECT_COMMAND_H

#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<sstream>
#include<iostream>
#include "ros/ros.h"

namespace Kv01 {

// data pre prikazy unknow command / OK a time out
#define OK_data1 0x4F
#define OK_data2 0x4B

#define TO_data1 0x54
#define TO_data2 0x4F

#define UC_data1 0x55
#define UC_data2 0x43



//prikazy na posielanie pre servis Driver spolocne s datami

#define comm_status  		0xAA

#define comm_brzdy		0xBB
#define comm_brzdy_run		0xB1
#define comm_brzdy_stop 	0xB0

#define comm_gripper_control    0xD5

#define comm_voltages 		0xCC

#define comm_message		0xDD

#define comm_motory_en	  0xD1
#define comm_motory_dis	  0xD0
#define comm_motory_speed       0xD2
#define comm_position           0xD3

#define comm_err_ok	 	0x11
#define comm_err_communication 	0x22
#define comm_err_command       	0x33
#define comm_err_data       	0x44

#define comm_central_stop_wifi	0xFF

#define comm_response_ok	0x11
#define comm_response_bad	0x00


    class Command {
    private:
         uint8_t calculateCRC(uint8_t *buffer, unsigned int num);
    public:

    };
}



#endif //PROJECT_COMMAND_H
