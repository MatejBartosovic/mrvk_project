//
// Created by jakub on 30.5.2019.
//

#include "../include/mrvk_gui_interface/GuiInterface.h"
#include "../../../osm_planner/include/osm_planner/coordinates_converters/coordinates_converter_base.h"

#define SUCCESS "Success"
#define SET_SUCCESS_RESPONSE(res) res.message = SUCCESS; res.success = true;

namespace mrvk {
    GuiInterface::GuiInterface() : nh("~"),
                                   waypointsAs(nh, "process_waypoints",
                                                         boost::bind(&GuiInterface::waypointsASGoal, this, _1), false),
                                   moveBaseActionClient("move_base", true) {

        // read required param from server
        if (!nh.getParam(ORIGIN_LATITUDE_PARAM_PATH, latitudeOrigin)) {
            ROS_ERROR("Param %s missing on param server", ORIGIN_LATITUDE_PARAM_PATH);
        }
        if (!nh.getParam(ORIGIN_LONGTITUDE_PARAM_PATH, longitudeOrigin)) {
            ROS_ERROR("Param %s missing on param server", ORIGIN_LONGTITUDE_PARAM_PATH);
        }

        // check if move base action API running
        if (!moveBaseActionClient.waitForServer(ros::Duration(5))) {
            ROS_ERROR("move_base action server is not running");
            ros::shutdown();
        }

        waypointsAs.registerPreemptCallback(boost::bind(&GuiInterface::stopMovement, this));
        waypointsAs.start();

        // advertise services
        addWaypointSrv = nh.advertiseService("add_waypoint", &GuiInterface::addWaypoint, this);
        editWaypointSrv = nh.advertiseService("edit_waypoint", &GuiInterface::editWaypoint, this);
        eraseWaypointsQueueSrv = nh.advertiseService("erase_waypoints", &GuiInterface::eraseWaypointsQueue, this);
        getWaypointsSrv = nh.advertiseService("get_waypoints", &GuiInterface::getWaypointsQueue, this);
        swapWaypointsSrv = nh.advertiseService("swap_waypoints", &GuiInterface::swapWaypoints, this);
    }

    bool GuiInterface::addWaypoint(mrvk_gui_interface::AddGeoWaypointRequest& req,
                                   mrvk_gui_interface::AddGeoWaypointResponse& res) {

        if (req.index == -1) {
            // add waypoint to the end of queue
            waypoints.push_back(req.waypoint);
        } else if (req.index <= waypoints.size()) {
            waypoints.insert(waypoints.begin() + req.index, req.waypoint);
        } else {
            res.waypoints = waypoints;
            res.message = "Wrong argument: index. Out of range";
            return false;
        }
        res.waypoints = waypoints;
        SET_SUCCESS_RESPONSE(res);
        return true;
    }

    bool GuiInterface::eraseWaypointsQueue(mrvk_gui_interface::EraseWaypointsQueueRequest& req,
                                           mrvk_gui_interface::EraseWaypointsQueueResponse& res) {

        waypoints.clear();
        SET_SUCCESS_RESPONSE(res);
        return true;
    }

    void GuiInterface::pushBackWaypoint(mrvk_gui_interface::GeoPoint point) {
        geometry_msgs::Pose pose;
        osm_planner::coordinates_converters::GeoNode geoNode = {point.latitude, point.longitude, 0, 0};
        osm_planner::coordinates_converters::HaversineFormula converter;
        converter.setOrigin(latitudeOrigin, longitudeOrigin);
        pose.position.x = converter.getCoordinateX(geoNode);
        pose.position.y = converter.getCoordinateY(geoNode);
        pose.position.z = 0;

        waypointsQueue.push_back(pose);

        ROS_DEBUG("Add waypoint: GEO[lat,long]: [%f, %f], MAP[X,Y]: [%f, %f]",
                  point.latitude, point.longitude, pose.position.x, pose.position.y);

    }

    bool GuiInterface::getWaypointsQueue(mrvk_gui_interface::GetWaypointsQueueRequest& req,
                                         mrvk_gui_interface::GetWaypointsQueueResponse& res) {

        res.waypoints = waypoints;
        SET_SUCCESS_RESPONSE(res);
        return true;
    }

    bool GuiInterface::editWaypoint(mrvk_gui_interface::EditWaypointRequest& req,
                                    mrvk_gui_interface::EditWaypointResponse& res) {
        if (req.index < waypoints.size()) {
            waypoints[req.index] = req.waypoint;
        } else {
            res.waypoints = waypoints;
            res.message = "Wrong argument: index. Out of range";
            res.success = false;
            return false;
        }

        res.waypoints = waypoints;
        SET_SUCCESS_RESPONSE(res);
        return true;
    }

    bool GuiInterface::swapWaypoints(mrvk_gui_interface::SwapWaypointsRequest& req,
                                     mrvk_gui_interface::SwapWaypointsResponse& res) {

        if (req.index1 < 0 ||  waypoints.size() <= req.index1
                || req.index2 < 0 ||  waypoints.size() <= req.index2) {

            res.waypoints = waypoints;
            res.message = "Wrong argument: index. Out of range";
            res.success = false;
            return false;
        }
        std::iter_swap(waypoints.begin() + req.index1, waypoints.begin() + req.index2);
        res.waypoints = waypoints;
        SET_SUCCESS_RESPONSE(res);
        return true;
    }

    void GuiInterface::waypointsASGoal(const mrvk_gui_interface::PerformWaypointsGoalConstPtr& actionGoal) {
        waypointsCount = waypointsQueue.size();
        performingWaypoint = 0;

        for (auto point: waypointsQueue) {
            move_base_msgs::MoveBaseGoal goalPoint;
            goalPoint.target_pose.header.stamp = ros::Time::now();
            goalPoint.target_pose.header.frame_id = TF_WORLD_FRAME;
            goalPoint.target_pose.pose = point;

            moveBaseActionClient.sendGoal(goalPoint, boost::bind(&GuiInterface::moveBaseGoalDone, this, _1, _2),
                                          boost::bind(&GuiInterface::moveBaseActive, this),
                                          boost::bind(&GuiInterface::moveBaseFeedback, this, _1));
            performingWaypoint++;
            moveBaseActionClient.waitForResult();

            if (moveBaseActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Way point has been successful achieved");
            } else {
                ROS_ERROR("Way point has not been achieved");
            }
        }

        // send result
        mrvk_gui_interface::PerformWaypointsResult result;
        result.message = "Success";
        result.success = true;
        waypointsAs.setSucceeded(result);
    }

    void GuiInterface::stopMovement() {
        moveBaseActionClient.cancelAllGoals();
    }

    void GuiInterface::moveBaseGoalDone(const actionlib::SimpleClientGoalState& state,
                                        const move_base_msgs::MoveBaseResultConstPtr& result) {

        using namespace actionlib;

        switch(state.state_) {
            case SimpleClientGoalState::ACTIVE:
            case SimpleClientGoalState::SUCCEEDED:
                break;
            // some error is occured
            default:
                mrvk_gui_interface::PerformWaypointsResult resultMsg;
                resultMsg.message = state.getText();
                resultMsg.success = false;
                waypointsAs.setSucceeded(resultMsg);
        }

        if (!waypointsQueue.empty()) {
            waypointsQueue.pop_front();
        }

    }

    void GuiInterface::moveBaseFeedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        waypointsAsFeedback.performing_waypoint = performingWaypoint;
        waypointsAsFeedback.waypoints_count = waypointsCount;
        auto robotPose = feedback->base_position;
        waypointsAsFeedback.base_position = robotPose;
        waypointsAsFeedback.remaining_distance = distanceBetweenPoses(waypointsQueue.front(), robotPose.pose);
        // copy list to vector
        waypointsAsFeedback.poses.poses = std::vector<geometry_msgs::Pose> {std::begin(waypointsQueue), std::end(waypointsQueue)};

        waypointsAs.publishFeedback(waypointsAsFeedback);
    }

    void GuiInterface::moveBaseActive() {

    }

    double GuiInterface::distanceBetweenPoses(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
        return sqrt(pow(p1.position.x - p2.position.x, 2)
                    + pow(p1.position.y - p2.position.y, 2)
                    + pow(p1.position.z - p2.position.z, 2));
    }


}