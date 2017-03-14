#include <ros/ros.h>
#include <iostream>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

#include"sensor_msgs/JointState.h"
#include "kv01_driver/joint_speed.h"
#include "kv01_driver/common_functions.h"
#include "kv01_actionserver/moveAction.h"

class Kv01Action
{
protected:

  ros::NodeHandle nh_;
  int vzorky,vzorky_real,goal_time;
  bool flag;
  
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; 
  trajectory_msgs::JointTrajectory goal_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  std::string action_name_;
  ros::Duration goal_time_duration;
  
  ros::Subscriber subs_position;
  ros::Publisher speed_pub;
  
public:

  Kv01Action(std::string name):
  as_(nh_,"/kv01/arm_controller/follow_joint_trajectory",/* boost::bind(&Kv01Action::executeCB, this, _1),*/ false),
  action_name_("/kv01/arm_controller/follow_joint_trajectory")
  {    
    vzorky_real=-2;
    as_.registerGoalCallback(boost::bind(&Kv01Action::goalCB,this));
    as_.registerPreemptCallback(boost::bind(&Kv01Action::preemptCB,this));
    
    subs_position = nh_.subscribe("/kv01/joint_states", 1, &Kv01Action::analysisCB,this);
    speed_pub = nh_.advertise<kv01_driver::joint_speed>("/kv01/joint_speed", 12);
    
    as_.start();   
  }

  ~Kv01Action(void)
  {
  }
  
  bool speed_pos_pub(int joint,double speed,double pos)
  {
    kv01_driver::joint_speed joint_speed;
    joint_speed.command = comm_position;
    joint_speed.joint = joint; 
    joint_speed.speed = speed;
    joint_speed.position = pos;
    speed_pub.publish(joint_speed);
  }
  
  void goalCB()
  {
    vzorky_real=1;
    flag=false;
    ros::Time goal_time = ros::Time::now();
    goal_time_duration=ros::Duration(goal_time.sec,goal_time.nsec);
    goal_=as_.acceptNewGoal()->trajectory;
  }
  
  void preemptCB()
  {
    ROS_INFO("Nová akcia predbieha predchádzajúcu skorej ako skoncila");
    as_.setPreempted();
  }
  
  void analysisCB(const sensor_msgs::JointState::ConstPtr& arm)
  {
  
    if (!as_.isActive())
      return;

    ros::Time now = ros::Time::now();
    
    feedback_.header.stamp = now;

    feedback_.joint_names.resize(6);
    feedback_.actual.positions.resize(6);
    feedback_.actual.velocities.resize(6);

    for (int i = 0; i < 6; i++)
    {
      switch(i)
      {
	case 0:
	  feedback_.joint_names[i] = "klb0";
          feedback_.actual.positions[i] = arm->position[i];
          //ros::param::get("/speedklb0",feedback_.actual.velocities[i]);
          break;
	case 1:
          feedback_.joint_names[i] = "klb1";
          feedback_.actual.positions[i] = arm->position[i];
          //ros::param::get("/speedklb1",feedback_.actual.velocities[i]);
          break;
	case 2:
          feedback_.joint_names[i] = "klb2";
          feedback_.actual.positions[i] = arm->position[i];
          //ros::param::get("/speedklb2",feedback_.actual.velocities[i]);
          break;
	case 3:
          feedback_.joint_names[i] = "klb3";
          feedback_.actual.positions[i] = arm->position[i];
          //ros::param::get("/speedklb3",feedback_.actual.velocities[i]);
          break;
	case 4:
          feedback_.joint_names[i] = "klb4";
          feedback_.actual.positions[i] = arm->position[i];
          //ros::param::get("/speedklb4",feedback_.actual.velocities[i]);
          break;
	case 5:
          feedback_.joint_names[i] = "klb5";
          feedback_.actual.positions[i] = arm->position[i];
          //ros::param::get("/speedklb5",feedback_.actual.velocities[i]);
        break;
       }
    }
        
    if(vzorky_real != -2)
      
      vzorky = goal_.points.size();
      ROS_INFO("VZORKY %d",vzorky);
    
    {
     /* vzorky = goal_.points.size();
      if (vzorky_real == 0 || (((feedback_.actual.positions[0] <= (goal_.points[vzorky_real - 1].positions[0] + 0.1)) && (feedback_.actual.positions[0] >= (goal_.points[vzorky_real - 1].positions[0] - 0.1)))
                           &&  ((feedback_.actual.positions[1] <= (goal_.points[vzorky_real - 1].positions[1] + 0.1)) && (feedback_.actual.positions[1] >= (goal_.points[vzorky_real - 1].positions[1] - 0.1)))
                           &&  ((feedback_.actual.positions[2] <= (goal_.points[vzorky_real - 1].positions[2] + 0.1)) && (feedback_.actual.positions[2] >= (goal_.points[vzorky_real - 1].positions[2] - 0.1)))
                           &&  ((feedback_.actual.positions[3] <= (goal_.points[vzorky_real - 1].positions[3] + 0.1)) && (feedback_.actual.positions[3] >= (goal_.points[vzorky_real - 1].positions[3] - 0.1)))
                           &&  ((feedback_.actual.positions[4] <= (goal_.points[vzorky_real - 1].positions[4] + 0.1)) && (feedback_.actual.positions[4] >= (goal_.points[vzorky_real - 1].positions[4] - 0.1)))      
                           &&  ((feedback_.actual.positions[5] <= (goal_.points[vzorky_real - 1].positions[5] + 0.1)) && (feedback_.actual.positions[5] >= (goal_.points[vzorky_real - 1].positions[5] - 0.1)))))
      {*/
	for (int i = 0; i < 6; i++)// to_do dorobit kontrolu case pre aktualny cas a realny cas simulacie
	{
	  switch (i)
	  {
	    case 0: 
	    {
	      speed_pos_pub(i,goal_.points[vzorky-2].velocities[i],goal_.points[vzorky-1].positions[i]);
	      break;
	    }
	   case 1: 
	    {
	      speed_pos_pub(i,goal_.points[vzorky-2].velocities[i],goal_.points[vzorky-1].positions[i]);
	      break;
	    }
	   case 2: 
	    {
	      speed_pos_pub(i,goal_.points[vzorky-2].velocities[i],goal_.points[vzorky-1].positions[i]);
	      break;
	    }
	   case 3: 
	    {
	      speed_pos_pub(i,goal_.points[vzorky-2].velocities[i],goal_.points[vzorky-1].positions[i]);
	      break;
	    }
	   case 4: 
	    {
	      speed_pos_pub(i,goal_.points[vzorky-2].velocities[i],goal_.points[vzorky-1].positions[i]);
	      break;
	    }
	   case 5: 
	    {
	      speed_pos_pub(i,goal_.points[vzorky-2].velocities[i],goal_.points[vzorky-1].positions[i]);
	      break;
	    }
	  }  
	}
	vzorky_real = -2;
	result_.error_code =0;
        as_.setSucceeded(result_);  
      }/*
      else
      /*{
	vzorky_real--;
      }
      ros::Duration duration(1.0);
      
      ros::Time now = ros::Time::now();
      ros::Duration now_duration(now.sec,now.nsec);
      
      ros::Duration aktual = now_duration - goal_time_duration;
      
      ros::Duration goal_now(goal_.points[vzorky_real].time_from_start.sec, goal_.points[vzorky_real].time_from_start.nsec);
      
      if ((aktual > (goal_now + duration)) || (aktual < (goal_now - duration)) || flag)
      {
	result_.error_code = -1;
	as_.setAborted(result_); // to_DO doplnit vypnutie vsetkych motorov
        request_for_driver(0,comm_message,comm_motory_speed,0,0);
	request_for_driver(1,comm_message,comm_motory_speed,0,0);
	request_for_driver(2,comm_message,comm_motory_speed,0,0);
	request_for_driver(3,comm_message,comm_motory_speed,0,0);
	request_for_driver(4,comm_message,comm_motory_speed,0,0);
	request_for_driver(5,comm_message,comm_motory_speed,0,0);
      }
      feedback_.desired = goal_.points[vzorky_real];
      feedback_.error.positions.resize(6);
      
      for (int j = 0; j < 6; j++)
      {
	feedback_.error.positions[j] = feedback_.desired.positions[j] - feedback_.actual.positions[j];
      }

      vzorky_real++;
      
    }
    else
    {
      
      result_.error_code =0;
      as_.setSucceeded(result_);
    }
    as_.publishFeedback(feedback_);*/
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Kv01_Action");

  Kv01Action kv01(ros::this_node::getName());
  ros::spin();
  return 0;
}