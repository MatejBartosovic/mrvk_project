//
// Created by matejko on 11.3.2019.
//

#include <ros/ros.h>

class MrvkControllButtons{
public:
    MrvkControllButtons(){

    }

    ~MrvkControllButtons(){

    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "buttonTest");
    MrvkControllButtons buttons;

}