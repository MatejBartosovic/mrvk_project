//
// Created by mrvk on 17.8.2017.
//

#ifndef PROJECT_MRVK_SERIAL_H
#define PROJECT_MRVK_SERIAL_H

#include <serial/serial.h>

class MrvkBoard {
public:
    MrvkBoard(std::shared_ptr<serial::Serial> board, std::string name,int flag) :board(board), name(name),flag(flag) {
    }
    std::shared_ptr<serial::Serial> board;
    std::string name;
    int flag;
};

class MrvkSerial {
public:
    MrvkSerial() : flagsSum(0){}
    void addBoard(std::shared_ptr<serial::Serial> board, std::string name){
        boards.push_back(std::shared_ptr<MrvkBoard>(new MrvkBoard (board,name,1<<boards.size())));
    }
    bool init(int sb,int p,int bs){
        std::vector<int> fds;

    for(int i =0;i<boards.size();i++){
        if (!boards[i]->board->isOpen()){
            ROS_ERROR("port sa neotvoril");
            return false;
        }

        try{
            setupPort(sb, p, bs);
            boards[i]->board->flush();

        }catch (std::exception& e){
            ROS_ERROR("nenastavili sa parametre portu");
            return false;
        }
        fds.push_back(boards[i]->board->getFd());
    }
    fd_max = *std::max_element(fds.begin(),fds.end());
    for(int i=0;i<boards.size();i++){
        flagsSum +=boards[i]->flag;
    }
    return true;
    }
    int size(){
        return boards.size();
    }
    void setupPort(int sb,int p,int bs){

        /*nastavenie defaultnych parametrov*/
        //sb - stop bits
        //p  - parity
        //bs - byte size

        serial::stopbits_t stopBits;
        serial::parity_t parity;
        serial::bytesize_t byteSize;


        /*konvertovanie parametrov aby sa dali nastavit v serial class*/
        switch (sb)
        {
            case 1: stopBits = serial::stopbits_one;
                break;
            case 2: stopBits = serial::stopbits_two;
                break;
            default: stopBits = serial::stopbits_one_point_five;
                break;
        }
        switch (p)
        {
            case 0: parity = serial::parity_none;
                break;
            case 1: parity = serial::parity_odd;
                break;
            case 2: parity = serial::parity_even;
                break;
        }
        switch (bs)
        {
            case 5: byteSize = serial::fivebits;
                break;
            case 6: byteSize = serial::sixbits;
                break;
            case 7: byteSize = serial::sevenbits;
                break;
            case 8: byteSize = serial::eightbits;
                break;
        }
        /*samotne setnutie parametrov*/
        for(int i=0;i<boards.size();i++){
            boards[i]->board->setStopbits(stopBits);
            boards[i]->board->setParity(parity);
            boards[i]->board->setBytesize(byteSize);
        }
    }
    void close(){
        for(int i=0;i<boards.size();i++) {
            boards[i]->board->close();
        }
    }

    std::vector<std::shared_ptr<MrvkBoard>> boards;
    int fd_max;
    int flagsSum;
};

#endif //PROJECT_MRVK_SERIAL_H
