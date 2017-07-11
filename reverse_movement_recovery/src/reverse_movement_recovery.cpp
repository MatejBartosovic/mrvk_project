//
// Created by matejko on 11.7.2017.
//

#include "reverse_movement_recovery/reverse_movement_recovery.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(reverse_movement_recovery, ReverseMovementRecovery, reverse_movement_recovery::ReverseMovementRecovery, nav_core::RecoveryBehavior)

namespace reverse_movement_recovery{
    ReverseMovementRecovery::ReverseMovementRecovery(){

    }

    void ReverseMovementRecovery::initialize(std::string name, tf::TransformListener* tf,
            costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){

    }

    void ReverseMovementRecovery::runBehavior(){

    }
};