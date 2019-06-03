//
// Created by jakub on 30.5.2019.
//

#include <ros/ros.h>
#include <mrvk_gui_interface/GuiInterface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mrvk_gui_interface");
    ros::NodeHandle nh;
    mrvk::GuiInterface guiInterface;

    ros::spin();
    ros::shutdown();
    return 0;
}