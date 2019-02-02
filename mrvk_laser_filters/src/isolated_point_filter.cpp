//
// Created by adam on 31.1.2019.
//

#include "isolated_point_filter.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(isolated_point_filter::IsolatedPointFilter, filters::FilterBase<sensor_msgs::LaserScan>);

namespace isolated_point_filter {

IsolatedPointFilter::IsolatedPointFilter():
        min_accept_size_(1){
    //
}

IsolatedPointFilter::~IsolatedPointFilter() {

}

bool IsolatedPointFilter::configure() {
    getParam("min_accept_size", min_accept_size_);

    ROS_WARN("IsolatedPointFilter min_accept_size = %d", min_accept_size_);
}

/** \brief Update the filter and get the response
 * \param scan_in The new scan to filter
 * \param scan_out The filtered scan
 */
bool IsolatedPointFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
    scan_out = scan_in;
    int start_i = 0;
    int end_i = 0;
    int count = 0;
    bool is_region = false;

    for(int i = 0; i<scan_out.ranges.size(); i++){
        float dist = scan_out.ranges.at(i);
        if(!isinf(dist) && !isnan(dist)){
            count += 1;
            if (is_region == false) {
                is_region = true;
                start_i = i;
            }
        }
        else {
            if(is_region == true) {
                is_region = false;
                end_i = i;
                if (count < min_accept_size_) {
                    for(int j = start_i; i<end_i; i++) {
//                        ranges[j] = np.inf
                        scan_out.ranges.at(i) = INFINITY;
                    }
                }
                count = 0;
            }
        }

    }
    return true;
}

}

/*
 * TODO:
 * */
