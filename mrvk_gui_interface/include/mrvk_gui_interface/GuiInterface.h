//
// Created by jakub on 30.5.2019.
//

#ifndef SRC_GUIINTERFACE_H
#define SRC_GUIINTERFACE_H


#include <ros/ros.h>
#include <list>
#include <vector>
#include <cmath>


#include <mrvk_gui_interface/AddGeoWaypoint.h>
#include <mrvk_gui_interface/AddGeoWaypoints.h>
#include <mrvk_gui_interface/EraseWaypointsQueue.h>
#include <mrvk_gui_interface/GeoPoint.h>
#include <mrvk_gui_interface/GetWaypointsQueue.h>
#include <mrvk_gui_interface/PerformWaypointsAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <osm_planner/coordinates_converters/haversine_formula.h>


#include <move_base_msgs/MoveBaseAction.h>


#include <geometry_msgs/Pose.h>


#define ORIGIN_LATITUDE_PARAM_PATH "/move_base/Planner/origin_latitude"
#define ORIGIN_LONGTITUDE_PARAM_PATH "/move_base/Planner/origin_longitude"

#define TF_WORLD_FRAME "world"
#define TF_BASE_LINK_FRAME "base_link"


namespace mrvk {

    class GuiInterface {
    public:
        GuiInterface();

    private:
        ros::NodeHandle nh;

        ros::ServiceServer addWaypointSrv;
        ros::ServiceServer addWaypointsSrv;
        ros::ServiceServer eraseWaypointsQueueSrv;
        ros::ServiceServer getWaypointsSrv;


        bool addWaypoint(mrvk_gui_interface::AddGeoWaypointRequest &req, mrvk_gui_interface::AddGeoWaypointResponse &res);
        bool addWaypoints(mrvk_gui_interface::AddGeoWaypointsRequest &req, mrvk_gui_interface::AddGeoWaypointsResponse &res);
        bool eraseWaypointsQueue(mrvk_gui_interface::EraseWaypointsQueueRequest &req, mrvk_gui_interface::EraseWaypointsQueueResponse &res);
        bool getWaypointsQueue(mrvk_gui_interface::GetWaypointsQueueRequest &req, mrvk_gui_interface::GetWaypointsQueueResponse &res);

        // perform waypoints action server
        actionlib::SimpleActionServer<mrvk_gui_interface::PerformWaypointsAction> waypointsAs;
        mrvk_gui_interface::PerformWaypointsFeedback waypointsAsFeedback;
        void waypointsAsGoal(const mrvk_gui_interface::PerformWaypointsGoalConstPtr& actionGoal);
        void stopMovement();

        // move base action client
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient;
        void moveBaseGoalDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
        void moveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
        void moveBaseActive();






        void pushBackWaypoint(mrvk_gui_interface::GeoPoint point);
        double distanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);


        std::list<geometry_msgs::Pose> waypointsQueue;
        int waypointsCount;
        int performingWaypoint;

        double latitudeOrigin;
        double longitudeOrigin;



    };

}

#endif //SRC_GUIINTERFACE_H
