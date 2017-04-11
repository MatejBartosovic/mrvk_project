//
// Created by root on 4/4/17.
//
#include <stdio.h>
#include <stdint.h>
#include <ros/ros.h>
#include "../include/kv01_driver/command.h"
namespace Kv01 {

    uint8_t Command::calculateCRC(uint8_t *buffer, unsigned int num) {

        uint8_t CRC = 0x00;
        uint8_t A = 0, B = 0, C = 0, i = 0, j = 0, k = 0;
        for (i = 0; i < num; i++) {
            A = buffer[i];
            for (k = 0; k < 8; k++) {
                C = A ^ B;

                if (C & 0x01) {
                    C = ((B ^ 0x18) / 2) | 0x80;
                } else {
                    C = B / 2;
                }

                B = C;
                if (A & 0x01) {
                    A = (A / 2) | 0x80;
                } else {
                    A = A / 2;
                }
            }
        }
        CRC = B;

        if (CRC == 0xf0) {
            CRC = 0xf1;
        }
        return CRC;
    }
}
