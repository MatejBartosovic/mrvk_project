//
// Created by matejko on 14.10.2018.
//

#ifndef PROJECT_SERIALMESSAGE_H
#define PROJECT_SERIALMESSAGE_H

#define SERIAL_HEADER_START_BYTE 0xAA
#define SERIAL_FOOTER_END_BYTE 0xBB

namespace Adis16488{
    typedef struct __attribute__((packed)){
        //Header() : start(SERIAL_START_BYTE){}
        uint8_t start;
        uint8_t size;
    } Header;

    typedef struct __attribute__((packed)){
        //Header() : start(SERIAL_START_BYTE){}
        uint8_t end;
    } Footer;
}

#endif //PROJECT_SERIALMESSAGE_H