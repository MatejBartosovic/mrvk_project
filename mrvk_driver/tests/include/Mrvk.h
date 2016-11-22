/*
 * hw_interface.h
 *
 *  Created on: 28.9.2016
 *      Author: matejko
 */

#ifndef HW_INTERFACE_H_
#define HW_INTERFACE_H_
#include <joint_limits_interface/joint_limits_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
namespace Mrvk{

class Mrvk : public hardware_interface::RobotHW
{
public:
	Mrvk();
	bool init();
	void read();
	void write();
	ros::Time get_time();
	ros::Duration get_period();
	int get_frequency();
	//hardware interface variables
	std::vector<double> camera_pos;
	std::vector<double> camera_vel;
	std::vector<double> camera_eff;
	std::vector<double> camera_cmd;
	std::vector<double> act_wheel_pos;
	std::vector<double> act_wheel_vel;
	std::vector<double> act_wheel_eff;
	std::vector<double> act_wheel_cmd;
	std::vector<double> joint_wheel_pos;
	std::vector<double> joint_wheel_vel;
	std::vector<double> joint_wheel_eff;
	std::vector<double> joint_wheel_cmd;

	joint_limits_interface::VelocityJointSaturationInterface jnt_limit_interface;
	transmission_interface::ActuatorToJointStateInterface act_to_joint_interface;
	transmission_interface::JointToActuatorVelocityInterface joint_to_act_interface;
private:
	//hardware interfaces
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::VelocityJointInterface jnt_vel_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;

	//joint limit interface
	std::vector<joint_limits_interface::JointLimits> wheel_limit;
	std::vector<joint_limits_interface::JointLimits> camera_limit;

	//transmission interface
	std::vector<transmission_interface::JointData> joint_state_data;
	std::vector<transmission_interface::JointData> joint_cmd_data;
	std::vector<transmission_interface::ActuatorData> act_state_data;
	std::vector<transmission_interface::ActuatorData> act_cmd_data;
	std::vector<transmission_interface::SimpleTransmission> transmissions;

	double dim;
	ros::Time last;
	ros::Time now;
	//parameters
	int frequency;
	std::vector<std::string> wheel_joint;
	std::vector<std::string> camera_joint;
	std::vector<std::string> transmission_names;
	std::vector<double> transmission_ratio;
	std::string robot_description;

	//node handle
	ros::NodeHandle nh_param;
};
}




#endif /* HW_INTERFACE_H_ */
