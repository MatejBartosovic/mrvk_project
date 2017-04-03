//
// Created by matejko on 5.2.2017.
//

#include "kv01_driver/hwInterface.h"
#include <urdf/model.h>

namespace Kv01{
    HwInterface::HwInterface() : hardware_interface::RobotHW(), pos(6,0), vel(6,0),eff(6,0), pos_cmd(6,0), vel_cmd(6,0) {
        std::vector<std::string> joints;
        ros::param::get("kv01_joint_names", joints);

        if (joints.size() != 6)
            throw std::length_error("expected 6 joint on parameter server, found " + joints.size());

        //state interface
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[0], &pos[0], &vel[0], &eff[0]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[1], &pos[1], &vel[1], &eff[1]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[2], &pos[2], &vel[2], &eff[2]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[3], &pos[3], &vel[3], &eff[3]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[4], &pos[4], &vel[4], &eff[4]));
        jointStatedInterface.registerHandle(hardware_interface::JointStateHandle(joints[5], &pos[5], &vel[5], &eff[5]));


        //robot velocity interface
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[0]), &vel_cmd[0]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[1]), &vel_cmd[1]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[2]), &vel_cmd[2]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[3]), &vel_cmd[3]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[4]), &vel_cmd[4]));
        jointVelInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[5]), &vel_cmd[5]));

        //camera position interface
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[0]), &pos_cmd[0]));
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[1]), &pos_cmd[1]));
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[2]), &pos_cmd[2]));
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[3]), &pos_cmd[3]));
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[4]), &pos_cmd[4]));
        jointPosInterface.registerHandle(
                hardware_interface::JointHandle(jointStatedInterface.getHandle(joints[5]), &pos_cmd[5]));

        urdf::Model urdf;
        if (!urdf.initParam("/robot_description"))
            throw std::invalid_argument("robot_description not found on parameter server");


        std::vector<joint_limits_interface::JointLimits> joint_limits(6);
        for (int i = 0; i < 6; i++) {
            boost::shared_ptr<const urdf::Joint> urdf_joint = urdf.getJoint(joints[i]);

            //get limits from urdf
            if (urdf_joint) {
                getJointLimits(urdf_joint, joint_limits[i]);
            }
            //get limits from parameter server
            joint_limits_interface::getJointLimits(joints[i], ros::NodeHandle("/"), joint_limits[i]);
        }

        for (int i = 0; i < 6; i++) {
            //robot velocity limit
            robotVelocityLimitInterface.registerHandle(
                    joint_limits_interface::VelocityJointSaturationHandle(jointVelInterface.getHandle(joints[i]),
                                                                          joint_limits[i]));
        }


        registerInterface(&jointStatedInterface);
        registerInterface(&jointVelInterface);
        registerInterface(&jointPosInterface);
        //enforceLimits();
    }
}
