//
// Created by matejko on 8.3.2017.
//

#ifndef PROJECT_VFH_PLANNER_ROS_H
#define PROJECT_VFH_PLANNER_ROS_H

#include <nav_core/base_local_planner.h>


namespace vfh_local_planner {
    /**
     * @class VFHPlannerROS
     * @brief ROS Wrapper for the VFHPlanner that adheres to the
     * BaseLocalPlanner interface and can be used as a plugin for move_base.
     */
    class VFHPlusPlannerROS : public nav_core::BaseLocalPlanner {
    public:
        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        virtual bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param orig_global_plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief  Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform listener
         * @param costmap_ros The cost map to use for assigning costs to local plans
              */
        virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;
    };
}



#endif //PROJECT_VFH_PLANNER_ROS_H
