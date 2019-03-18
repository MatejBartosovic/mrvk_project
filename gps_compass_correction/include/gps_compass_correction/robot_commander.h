//
// Created by controller on 10/28/18.
//

#ifndef PROJECT_ROBOT_COMMANDER_H
#define PROJECT_ROBOT_COMMANDER_H

#include <ros/ros.h>

//!  A RobotCommander class
/*!
  A Class for forward moving with Robot using cmd_vel publisher
  It's necessary for auto_compute_bearing service
*/

class RobotCommander{
    public:
    /**
     * @brief Create cmd_vel publisher
     * @param n - ros node handler
     */
    RobotCommander(ros::NodeHandle &n);

    /**
     * @brief Publish velocity_ during moving_time_
     */
    void move();

private:

    ros::Publisher cmd_vel_pub_;        ///< Publisher
    double moving_time_;                ///< Publishing time
    double velocity_;                   ///< Published velocity
    const double PUBLISHER_RATE_ = 5.0; ///< Publishing rate
};

#endif //PROJECT_ROBOT_COMMANDER_H
