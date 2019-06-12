//
// Created by jakub on 30.5.2019.
//

#ifndef SRC_GUIINTERFACE_H
#define SRC_GUIINTERFACE_H

// ros
#include <ros/ros.h>

// std
#include <list>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cstdlib>

// msgs
#include <mrvk_gui_interface/GeoPoint.h>
#include <mrvk_gui_interface/AddGeoWaypoint.h>
#include <mrvk_gui_interface/ClearWaypoints.h>
#include <mrvk_gui_interface/EraseWaypoint.h>
#include <mrvk_gui_interface/GetWaypoints.h>
#include <mrvk_gui_interface/EditWaypoint.h>
#include <mrvk_gui_interface/SwapWaypoints.h>
#include <geometry_msgs/Pose.h>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <mrvk_gui_interface/PerformWaypointsAction.h>
#include <move_base_msgs/MoveBaseAction.h>

// others
#include <osm_planner/coordinates_converters/haversine_formula.h>

// defines
#define ORIGIN_LATITUDE_PARAM_PATH "/move_base/Planner/origin_latitude"
#define ORIGIN_LONGTITUDE_PARAM_PATH "/move_base/Planner/origin_longitude"
#define TF_WORLD_FRAME "world"
#define TF_BASE_LINK_FRAME "base_link"
#define SUCCESS "Success"
#define SET_SUCCESS_RESPONSE(res) res.message = SUCCESS; res.success = true;


namespace mrvk {

    class GuiInterface {
    public:
        GuiInterface();

    private:
        ros::NodeHandle nh;

        ros::ServiceServer addWaypointSrv;
        ros::ServiceServer clearWaypointsSrv;
        ros::ServiceServer getWaypointsSrv;
        ros::ServiceServer editWaypointSrv;
        ros::ServiceServer swapWaypointsSrv;
        ros::ServiceServer eraseWaypointSrv;

        bool addWaypoint(mrvk_gui_interface::AddGeoWaypointRequest &req, mrvk_gui_interface::AddGeoWaypointResponse &res);
        bool editWaypoint(mrvk_gui_interface::EditWaypointRequest &req, mrvk_gui_interface::EditWaypointResponse &res);
        bool clearWaypoints(mrvk_gui_interface::ClearWaypointsRequest& req,
                            mrvk_gui_interface::ClearWaypointsResponse& res);
        bool getWaypoints(mrvk_gui_interface::GetWaypointsRequest& req,
                          mrvk_gui_interface::GetWaypointsResponse& res);
        bool swapWaypoints(mrvk_gui_interface::SwapWaypointsRequest &req, mrvk_gui_interface::SwapWaypointsResponse &res);
        bool eraseWaypoint(mrvk_gui_interface::EraseWaypointRequest &req, mrvk_gui_interface::EraseWaypointResponse &res);


        // perform waypoints action server
        actionlib::SimpleActionServer<mrvk_gui_interface::PerformWaypointsAction> waypointsAs;
        mrvk_gui_interface::PerformWaypointsFeedback waypointsAsFeedback;
        void waypointsASGoal(const mrvk_gui_interface::PerformWaypointsGoalConstPtr& actionGoal);
        void stopMovement();

        // move base action client
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseActionClient;
        void moveBaseGoalDone(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
        void moveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
        void moveBaseActive();

//        void pushBackWaypoint(mrvk_gui_interface::GeoPoint point);
        double distanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2);

        void saveWaypointsToFile();
        void loadWaypointsFromFile();

        std::string waypointsBackupFile;

        std::vector<mrvk_gui_interface::GeoPoint> waypoints;
        std::list<geometry_msgs::Pose> waypointsQueue;
        int waypointsCount;
        int performingWaypoint;

        double latitudeOrigin;
        double longitudeOrigin;

    };
}

#endif //SRC_GUIINTERFACE_H
