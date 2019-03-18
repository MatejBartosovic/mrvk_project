//
// Created by matejko on 18.3.2019.
//

#ifndef PROJECT_MRVKCONTROLLBUTTONS_H
#define PROJECT_MRVKCONTROLLBUTTONS_H

#include <ros/ros.h>
#include <exception>
class MrvkControllButtonsException : std::exception{
public:
    MrvkControllButtonsException(const std::string& str) : message(str){

    }
    virtual const char* what(){
        return message.c_str();
    }
    std::string message;
};

namespace mrvk_gui {
    class MrvkControllButtons {
    public:
        MrvkControllButtons();
        void setCentralStop();
        void resetCentralStop();
        void blockMovement();
        void unblockMovement();

    private:
        ros::ServiceClient setCentralStopServiceClient;
        ros::ServiceClient resetCentralStopServiceClient;
        ros::ServiceClient blockMovementServiceClient;
        ros::ServiceClient unblockMovementServiceClient;
    };
}

#endif //PROJECT_MRVKCONTROLLBUTTONS_H
