//
// Created by adam on 31.1.2019.
//

#ifndef MRVK_PROJECT_ISOLATEDPOINTFILTER_H
#define MRVK_PROJECT_ISOLATEDPOINTFILTER_H

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include "filters/filter_chain.h"

namespace isolated_point_filter {

class IsolatedPointFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
public:
    /** \brief Constructor
     * \param averaging_length How many scans to average over.
     */
    IsolatedPointFilter();
    ~IsolatedPointFilter();

    bool configure();

    /** \brief Update the filter and get the response
     * \param scan_in The new scan to filter
     * \param scan_out The filtered scan
     */
    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);


private:
    int min_accept_size_;
};

}

#endif //MRVK_PROJECT_ISOLATEDPOINTFILTER_H
