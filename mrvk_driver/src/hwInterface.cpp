//
// Created by matejko on 5.2.2017.
//

#include "mrvk_driver/hwInterface.h"
#include <urdf/model.h>

namespace Mrvk{
    HwInterface::HwInterface() : hardware_interface::RobotHW(), pos(4,0), vel(4,0),eff(4,0), pos_cmd(2,0), vel_cmd(2,0) {
        std::vector<std::string> joints;
        ros::param::get("mrvk_joint_names", joints);

        if (joints.size() != 4)
            throw std::length_error("expected 4 joint on parameter server, found " + joints.size());

        //state interface
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[0], &pos[0], &vel[0], &eff[0]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[1], &pos[1], &vel[1], &eff[1]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[2], &pos[2], &vel[2], &eff[2]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[3], &pos[3], &vel[3], &eff[3]));

        //robot velocity interface
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[0]), &vel_cmd[0]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[1]), &vel_cmd[1]));

        //camera position interface
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[2]), &pos_cmd[0]));
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[3]), &pos_cmd[1]));

        urdf::Model urdf;
        if (!urdf.initParam("/robot_description"))
            throw std::invalid_argument("robot_description not found on parameter server");


        std::vector<joint_limits_interface::JointLimits> joint_limits(4);
        for (int i = 0; i < 4; i++) {
            urdf::JointConstSharedPtr urdf_joint = urdf.getJoint(joints[i]);

            //get limits from urdf
            if (urdf_joint) {
                getJointLimits(urdf_joint, joint_limits[i]);
            }
            //get limits from parameter server
            joint_limits_interface::getJointLimits(joints[i], ros::NodeHandle("/"), joint_limits[i]);
        }

        for (int i = 0; i < 2; i++) {
            //robot velocity limit
            robotVelocityLimitInterface.registerHandle(
                    joint_limits_interface::VelocityJointSaturationHandle(jointVelInterface.getHandle(joints[i]),
                                                                          joint_limits[i]));
            //camera position limit
            cameraPositionLimitInterface.registerHandle(
                    joint_limits_interface::PositionJointSaturationHandle(jointPosInterface.getHandle(joints[i+2]),
                                                                          joint_limits[i+2]));
        }
        registerInterface(&jointStatedInterface);
        registerInterface(&jointVelInterface);
        registerInterface(&jointPosInterface);
        //enforceLimits();
    }
}
