//ROS libraries
#include <ros/ros.h>

//hardware_interface libraries
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

//joint limit interface libraries
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>

//URDF library
#include <urdf/model.h>

//C libraries
#include <signal.h>

//project library
#include <mrvk_driver/Mrvk.h>



namespace Mrvk{

	Mrvk::Mrvk(): dim(2),nh_param("~"),robot_description("/robot_description")
								  {
		act_wheel_pos.resize(dim,0.0);
		act_wheel_vel.resize(dim,0.0);
		act_wheel_eff.resize(dim,0.0);
		act_wheel_cmd.resize(dim,0.0);
		joint_wheel_pos.resize(dim,0.0);
		joint_wheel_vel.resize(dim,0.0);
		joint_wheel_eff.resize(dim,0.0);
		joint_wheel_cmd.resize(dim,0.0);

		camera_pos.resize(dim,0.0);
		camera_vel.resize(dim,0.0);
		camera_eff.resize(dim,0.0);
		camera_cmd.resize(dim,0.0);

		joint_state_data.resize(dim);
		act_state_data.resize(dim);
		joint_cmd_data.resize(dim);
		act_cmd_data.resize(dim);

		wheel_limit.resize(dim);
		camera_limit.resize(dim);

		nh_param.getParam("wheel_joints",wheel_joint);
		nh_param.getParam("camera_joints",camera_joint);
		nh_param.getParam("transmission_names",transmission_names);
		nh_param.getParam("transmission_ratio",transmission_ratio);
	    nh_param.param<int>("frequency", frequency, 10);
		}
	bool Mrvk::init(){
		//get ros parameters
		if((wheel_joint.size()!=dim)||(camera_joint.size()!=dim)||(transmission_names.size()!=dim)||(transmission_ratio.size()!=dim)){
			ROS_ERROR("wheel_joints(current size = %d ), camera_joints (current size = %d ), "
					"transmission_names (current size = %d ) and transmission_ratio (current size = %d ) "
					"are vectors with size 2!!!. Pleas set parameters properly",
					(int)wheel_joint.size(),(int)camera_joint.size(),(int)transmission_names.size(),(int)transmission_ratio.size());
			return false;
		}
		std::string robot_URDF;
		nh_param.param<std::string>(robot_description,robot_URDF,"");

		//load URDF
		urdf::Model urdf;
		if(!urdf.initString(robot_URDF)){
			ROS_ERROR("Unable to find robot description on parameter server EXITTINGG");
			return false;
		}
		for(int i=0;i<dim;i++){
			ROS_ERROR("zaciatok");
			//joint state interface
			jnt_state_interface.registerHandle(hardware_interface::JointStateHandle (wheel_joint[i],&joint_wheel_pos[i],&joint_wheel_vel[i],&joint_wheel_eff[i]));
			jnt_state_interface.registerHandle(hardware_interface::JointStateHandle (camera_joint[i],&camera_pos[i],&camera_vel[i],&camera_eff[i]));

			//joint velocity interface
			this->jnt_vel_interface.registerHandle(hardware_interface::JointHandle(jnt_state_interface.getHandle(wheel_joint[i]),&joint_wheel_cmd[i]));

			//joint position interface
			this->jnt_pos_interface.registerHandle(hardware_interface::JointHandle (jnt_state_interface.getHandle(camera_joint[i]),&camera_cmd[i]));

			//joint limit interface
			//podvozok
			boost::shared_ptr<const urdf::Joint> urdf_wheel_joint = urdf.getJoint(wheel_joint[i]);
			if(urdf_wheel_joint == NULL){
				ROS_ERROR("Failed to find '%s' in URDF ", wheel_joint[i].c_str());
				return false;
			}
			if(!getJointLimits(urdf_wheel_joint, wheel_limit[i])){
				ROS_ERROR("Failed to get '%s' limits from URDF",wheel_joint[i].c_str());
				return false;
			}
			//get acc limit from parameter server
			if(!joint_limits_interface::getJointLimits(wheel_joint[i], nh_param, wheel_limit[i])){
				ROS_ERROR("Failed to get '%s' joint limits from parameter server",wheel_joint[i].c_str());
				return false;
			}
			//kamera
			boost::shared_ptr<const urdf::Joint> urdf_camera_joint = urdf.getJoint(camera_joint[i]);
			if(urdf_camera_joint == NULL){
				ROS_ERROR("Failed to find '%s' in URDF ", camera_joint[i].c_str());
				return false;
			}
			if(!getJointLimits(urdf_camera_joint, camera_limit[i])){
				ROS_ERROR("Failed to get '%s' limits from URDF",camera_joint[i].c_str());
				return false;
			}
			jnt_limit_interface.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(jnt_vel_interface.getHandle(wheel_joint[i]),wheel_limit[i]));

			transmissions.push_back(transmission_interface::SimpleTransmission(transmission_ratio[i],0));
			joint_state_data[i].position.push_back(&joint_wheel_pos[i]);
			joint_state_data[i].velocity.push_back(&joint_wheel_vel[i]);
			joint_state_data[i].effort.push_back(&joint_wheel_eff[i]);
			joint_cmd_data[i].velocity.push_back(&joint_wheel_cmd[i]);

			act_state_data[i].position.push_back(&act_wheel_pos[i]);
			act_state_data[i].velocity.push_back(&act_wheel_vel[i]);
			act_state_data[i].effort.push_back(&act_wheel_eff[i]);
			act_cmd_data[i].velocity.push_back(&act_wheel_cmd[i]);

			ROS_ERROR(" i = %d",i);
			sleep(1);
			act_to_joint_interface.registerHandle(transmission_interface::ActuatorToJointStateHandle(transmission_names[i],&transmissions[1],act_state_data[i],joint_state_data[i]));
			sleep(1);
			joint_to_act_interface.registerHandle(transmission_interface::JointToActuatorVelocityHandle(transmission_names[i],&transmissions[1],act_cmd_data[i],joint_cmd_data[i]));

		}
		/*act_to_joint_interface.registerHandle(transmission_interface::ActuatorToJointStateHandle(transmission_names[0],&transmissions[0],act_state_data[0],joint_state_data[0]));
		joint_to_act_interface.registerHandle(transmission_interface::JointToActuatorVelocityHandle(transmission_names[0],&transmissions[0],act_cmd_data[0],joint_cmd_data[0]));

		act_to_joint_interface.registerHandle(transmission_interface::ActuatorToJointStateHandle(transmission_names[1],&transmissions[1],act_state_data[1],joint_state_data[1]));
		joint_to_act_interface.registerHandle(transmission_interface::JointToActuatorVelocityHandle(transmission_names[1],&transmissions[1],act_cmd_data[1],joint_cmd_data[1]));
*/
		this->registerInterface(&jnt_state_interface);
		this->registerInterface(&jnt_vel_interface);
		this->registerInterface(&jnt_pos_interface);
		return true;
	}
	void Mrvk::read(){
		/*
		 * TU treba nacitat do premennych act_wheel_pos, act_wheel_vel, act_wheel_eff
		 * ciste data z robota NICIM NENASOBIT!!!!!
		 * 		 * */
		 act_to_joint_interface.propagate(); //toto bude na konci readu
	}

	void Mrvk::write(){
		jnt_limit_interface.enforceLimits(get_period());
		joint_to_act_interface.propagate();
		/*
		 * TU len urob konverziu z double na int a posli act_wheel_cmd na zbernicu
		 * Ak je spravne nastaveny limit a prevodovka tak to nikdy nebude viac ako 1000
		 * */
		mrvk -> set Speed()
	}

	ros::Time Mrvk::get_time(){
		last = now;
		now = ros::Time::now();
		return now;
	}

	ros::Duration Mrvk::get_period(){
		return  now - last;
	}
	int Mrvk::get_frequency(){
		return frequency;
	}
}

void sigint_handler(int sig) {
		ROS_ERROR("Exiting...");
  		ros::shutdown();
  	}

int main(int argc, char **argv)
{
	//node init
	ros::init(argc, argv, "mrvk_driver");
	ros::NodeHandle n;
	signal(SIGINT, sigint_handler);

	//hw interface init
	Mrvk::Mrvk robot_hw;
	if(!robot_hw.init()){
		ROS_ERROR("EXITTING...");
		ros::shutdown();
		return 0;
	}

	//start controller manager
	controller_manager::ControllerManager cm(&robot_hw);

	//start spinner (pre servisi)
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//loop rate
	ros::Rate rate(robot_hw.get_frequency());
  while (n.ok())
  {
	  robot_hw.read();
	  cm.update(robot_hw.get_time(), robot_hw.get_period());
	  robot_hw.write();
	  ROS_ERROR("actR_cmd = %lf jointR_cmd = %lf",
			  robot_hw.act_wheel_cmd[1],robot_hw.joint_wheel_cmd[1]);
	  ROS_ERROR("act_vel = %lf, joint_vel %lf ",robot_hw.act_wheel_vel[1],robot_hw.joint_wheel_vel[1]);
	  robot_hw.act_wheel_vel[0] = robot_hw.act_wheel_cmd[0];
	  robot_hw.act_wheel_vel[1] = robot_hw.act_wheel_cmd[1];
	  rate.sleep();
  }
return 0;
}
