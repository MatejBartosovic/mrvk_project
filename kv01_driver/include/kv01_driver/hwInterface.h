//
// Created by matejko on 5.2.2017.
//

#ifndef PROJECT_HWINTERFACE_H
#define PROJECT_HWINTERFACE_H

//ros controll
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//joint limit interface libraries
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

namespace Kv01{
    class HwInterface : public hardware_interface::RobotHW{
    public:
        HwInterface();
        inline void enforceLimits(){
            current_time = ros::Time::now();
            robotVelocityLimitInterface.enforceLimits(current_time - last_time);
            cameraPositionLimitInterface.enforceLimits(current_time - last_time);
            last_time = current_time;
        }

        std::vector<double> pos;                                        //joint positions (0-5 robot)
        std::vector<double> vel;                                        //velocity (0-5 robot)
        std::vector<double> eff;                                        //effort (0-5 robot)
        std::vector<double> pos_cmd;                                    //position command to robot
        std::vector<double> vel_cmd;                                    //velocity command to robot

    private:
        hardware_interface::JointStateInterface jointStatedInterface;   //state interface
        hardware_interface::PositionJointInterface jointPosInterface;   //robot velocity interface
        hardware_interface::VelocityJointInterface jointVelInterface;   //camera position interface
        joint_limits_interface::JointLimits jointLimitInterface;        //limit interface

        joint_limits_interface::VelocityJointSaturationInterface robotVelocityLimitInterface;
        joint_limits_interface::PositionJointSaturationInterface cameraPositionLimitInterface;

        ros::Time last_time;
        ros::Time current_time;
    };

}


#endif //PROJECT_HWINTERFACE_H
