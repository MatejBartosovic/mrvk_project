//
// Created by matejko on 11.7.2017.
//

#ifndef PROJECT_REVERSE_MOVEMENT_RECOVERY_H
#define PROJECT_REVERSE_MOVEMENT_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
//#include <costmap_2d/costmap_layer.h>

namespace reverse_movement_recovery{
    /**
     * @class ReverseMovementRecovery
     * @brief A recovery behavior that move robot backward for specified ditance.
     */
    class ReverseMovementRecovery : public nav_core::RecoveryBehavior {
    public:
        /**
         * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
         * @param
         * @return
         */
        ReverseMovementRecovery();

        /**
         * @brief  Initialization function for the ReverseMovementRecovery recovery behavior
         * @param tf A pointer to a transform listener
         * @param global_costmap A pointer to the global_costmap used by the navigation stack
         * @param local_costmap A pointer to the local_costmap used by the navigation stack
         */
        void initialize(std::string name,tf2_ros::Buffer* tf,
                        costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap);

        /**
         * @brief  Run the ClearCostmapRecovery recovery behavior. Move robot backward for specified ditance.
         */
        void runBehavior();

    private:
        void pubVel(double vel = -0.2);
        costmap_2d::Costmap2DROS* global_costmap_, *local_costmap_;
        std::string name_;
        tf2_ros::Buffer* tf_;
        double distance_;
        double escape_vel_;
        bool initialized_;
        ros::Publisher vel_pub_;
    };
};


#endif //PROJECT_REVERSE_MOVEMENT_RECOVERY_H
