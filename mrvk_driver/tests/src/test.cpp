/*
 * test.cpp
 *
 *  Created on: 9.10.2016
 *      Author: matejko
 */

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <mrvk_driver/read_file.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <boost/foreach.hpp>

//joint limit interface libraries
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>

class transmission_test
{
public:
	transmission_test()
    : dim(3),
      act_names(3),
      act_pos(3, 0.0),
      act_vel(3, 0.0),
      act_eff(3, 0.0),
      act_pos_cmd(3, 0.0),
      act_vel_cmd(3, 0.0),
      act_eff_cmd(3, 0.0),
      robot_transmissions()
  {
    act_names[0] = "foo_actuator";
    act_names[1] = "bar_actuator";
    act_names[2] = "baz_actuator";

    // Populate actuators interface
    for (unsigned int i = 0; i < dim; ++i)
    {
    	ROS_ERROR("i  =  %d ",i);
      hardware_interface::ActuatorStateHandle state_handle(act_names[i], &act_pos[i], &act_vel[i], &act_eff[i]);
      act_state_iface.registerHandle(state_handle);

      hardware_interface::ActuatorHandle pos_cmd_handle(state_handle, &act_pos_cmd[i]);
      pos_act_iface.registerHandle(pos_cmd_handle);

      hardware_interface::ActuatorHandle vel_cmd_handle(state_handle, &act_vel_cmd[i]);
      vel_act_iface.registerHandle(vel_cmd_handle);

      hardware_interface::ActuatorHandle eff_cmd_handle(state_handle, &act_eff_cmd[i]);
      eff_act_iface.registerHandle(eff_cmd_handle);
    }
    robot_hw.registerInterface(&act_state_iface);
    robot_hw.registerInterface(&pos_act_iface);
    robot_hw.registerInterface(&vel_act_iface);
    robot_hw.registerInterface(&eff_act_iface);
  }
   void aaa(){

	  // Parse transmission info
	    const std::string urdf_filename = "urdf/transmission_interface_loader_valid.urdf";
	    std::string urdf;
	    transmission_interface::readFile(urdf_filename, urdf);

	    std::vector<transmission_interface::TransmissionInfo> infos = transmission_interface::parseUrdf(urdf_filename);
	    /*if(2 != infos.size())
	    	ROS_ERROR("neni 2");

	    // Get info for each transmission
	    const transmission_interface::TransmissionInfo& info_red = infos.front();
	    if(1 != info_red.actuators_.size()){
	    	ROS_ERROR("neni 1 1");
	    }
	    if(1 != info_red.joints_.size())
	    	ROS_ERROR("neni 1 2");



	    const transmission_interface::TransmissionInfo& info_diff = infos.back();
	    if(2 != info_diff.actuators_.size())
	    	ROS_ERROR("neni  2 1");
	    if(2 != info_diff.joints_.size())
	    	ROS_ERROR("neni 2 2");*/

	    // Load transmissions
	   // transmission_interface::TransmissionInterfaceLoader trans_iface_loader(&robot_hw, &robot_transmissions);
	    transmission_loader_.reset(new transmission_interface::TransmissionInterfaceLoader(&robot_hw, &robot_transmissions));
	    if(!transmission_loader_->load(urdf))
	    	ROS_ERROR("pica");
	    	// NOTE: Using URDF loader


	    // Actuator interfaces
	    hardware_interface::PositionActuatorInterface* act_pos_cmd_iface = robot_hw.get<hardware_interface::PositionActuatorInterface>();
	    hardware_interface::VelocityActuatorInterface* act_vel_cmd_iface = robot_hw.get<hardware_interface::VelocityActuatorInterface>();
	    hardware_interface::EffortActuatorInterface*   act_eff_cmd_iface = robot_hw.get<hardware_interface::EffortActuatorInterface>();
	    hardware_interface::JointStateInterface* a = robot_hw.get<hardware_interface::JointStateInterface>();
	    hardware_interface::VelocityJointInterface* b = robot_hw.get<hardware_interface::VelocityJointInterface>();

	    transmission_interface::JointToActuatorVelocityInterface* rrr = robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>();

	    if(rrr == NULL)
	    	ROS_ERROR("KOKOTINA NEJDE DO PICI");
	    else{
	    	ROS_ERROR("IDE TO!!!!!!!!!!!!!!~!~!~!");
	    	BOOST_FOREACH(std::string name, rrr->getNames()){
	    		ROS_ERROR("trans '%s'",name.c_str());
	    	}

	    }

	    if(NULL==robot_hw.get<transmission_interface::ActuatorToJointStateInterface>())
	    	ROS_ERROR("neni ActuatorToJointStateInterface");


	    if(!(act_pos_cmd_iface&&act_vel_cmd_iface&&act_eff_cmd_iface&&a))
	    	ROS_ERROR("hovenko");
	    BOOST_FOREACH( std::string name, a->getNames() ){
	    	ROS_ERROR("name = %s",name.c_str());
	    }
	    BOOST_FOREACH( std::string name, b->getNames() ){
	   	    	ROS_ERROR("name = %s",name.c_str());
	   	    }
	    ROS_ERROR("koniec");
  }

	ros::Time get_time(){
		return ros::Time::now();
	}
	hardware_interface::RobotHW   robot_hw;
	std::vector<double> act_pos, act_vel, act_eff, act_pos_cmd, act_vel_cmd, act_eff_cmd;
protected:
  unsigned int dim;
  std::vector<std::string> act_names;
  hardware_interface::ActuatorStateInterface    act_state_iface;
  hardware_interface::PositionActuatorInterface pos_act_iface;
  hardware_interface::VelocityActuatorInterface vel_act_iface;
  hardware_interface::EffortActuatorInterface   eff_act_iface;

  transmission_interface::RobotTransmissions robot_transmissions;

  boost::scoped_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;
};

int main (int argc, char **argv){
	ros::init(argc, argv, "aaa");
	ros::NodeHandle n;
	transmission_test test;
	test.aaa();
	controller_manager::ControllerManager cm(&test.robot_hw);
	ros::AsyncSpinner spinner(1);
		spinner.start();
	ros::Rate rate(1);
	while (n.ok())
	  {
		  ROS_ERROR("pred");
		  cm.update(test.get_time(), ros::Duration(1));
		  ROS_ERROR("za");
		  ROS_ERROR("pos = %lf, vel = %lf, eff = %lf cmd = %lf",test.act_pos[0],test.act_vel[0],test.act_eff[0],test.act_vel_cmd[0]);
		  test.act_vel[0] = test.act_vel_cmd[0];
		  rate.sleep();
		  ROS_ERROR("po");
	  }
	return 0;
}
