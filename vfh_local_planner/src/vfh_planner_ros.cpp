//
// Created by matejko on 8.3.2017.
//

#include <vfh_local_planner/vfh_planner_ros.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(vfh_local_planner::VFHPlusPlannerROS, nav_core::BaseLocalPlanner)

namespace vfh_local_planner {

    bool VFHPlusPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){}

    bool VFHPlusPlannerROS::isGoalReached(){}

    bool VFHPlusPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){}

    void VFHPlusPlannerROS::initialize(std::string name,  tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){}
}