//
// Created by controller on 10/25/18.
//

#include "gps_compass_correction/bearing_calculator.h"

BearingCalculator::BearingCalculator():
has_first_point_(false){

}

void BearingCalculator::addPoint(boost::shared_ptr<const sensor_msgs::NavSatFix> gps_data){

    first_point_.longitude = gps_data->longitude;
    first_point_.latitude = gps_data->latitude;
    has_first_point_ = true;
}


double BearingCalculator::calculate(boost::shared_ptr<const sensor_msgs::NavSatFix> gps_data){

    if (!has_first_point_){
        return NAN;
    }

    osm_planner::Parser::OSM_NODE second_point;
    second_point.longitude = gps_data->longitude;
    second_point.latitude = gps_data->latitude;
    has_first_point_ = false;
    return -osm_planner::Parser::Haversine::getBearing(first_point_, second_point);
}

bool BearingCalculator::hasFirstPoint() {
    return has_first_point_;
}