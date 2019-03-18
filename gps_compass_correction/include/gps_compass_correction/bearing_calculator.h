//
// Created by controller on 10/25/18.
//

#ifndef PROJECT_BEARING_CALCULATOR_H
#define PROJECT_BEARING_CALCULATOR_H

#include <osm_planner/osm_parser.h>

//!  A BearingCalculator class
/*!
  A Class for calculating angle between two GPS poses
  using Haversine formula from osm_parser.
  It's necessary for compute_bearing and auto_compute_bearing service
*/
class BearingCalculator{
public:
    BearingCalculator();

    /**
     * @brief Add first position from GPS
     * @param gps_data - corrected gps data
     * @return A true on success, false on error
     */
    void addPoint(boost::shared_ptr<const sensor_msgs::NavSatFix> gps_data);

    /**
     * @brief Add secont position from gps and
     * calculate angle between first and second point
     * It's use Haversine formula from osm_parser
     * @param gps_data - corrected gps data
     * @return A bearing angle if is both pose are correct
     * If are not correct then return NaN
     */
    double calculate(boost::shared_ptr<const sensor_msgs::NavSatFix> gps_data);

    /**
     * @brief Get information about successfully added first point
     * @return A boolean value
     */
    bool hasFirstPoint();

private:
    osm_planner::Parser::OSM_NODE first_point_;  ///< First pose in structure from osm_parser
    bool has_first_point_;                       ///< Just flag, if it's false then calculate method return NaN
};

#endif //PROJECT_BEARING_CALCULATOR_H
