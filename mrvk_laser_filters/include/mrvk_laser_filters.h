//
// Created by adam on 31.1.2019.
//

#ifndef MRVK_PROJECT_MRVKLASERFILTERS_H
#define MRVK_PROJECT_MRVKLASERFILTERS_H

#include <vector>
#include <cmath>

#include <boost/circular_buffer.hpp> // TODO continue here

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include "filters/filter_chain.h"

namespace mrvk_laser_filters {

class MrvkLaserFilters : public filters::FilterBase<sensor_msgs::LaserScan> {
public:
    /** \brief Constructor
     * \param averaging_length How many scans to average over.
     */
    MrvkLaserFilters();
    ~MrvkLaserFilters();

    bool configure();

    /** \brief Update the filter and get the response
     * \param scan_in The new scan to filter
     * \param scan_out The filtered scan
     */
    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

    /**
     *
     * @param scan_in
     * @param scan_out
     * @return
     */
    bool updateIsolatedPointsFilter(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);


private:
    bool enable_isolated_points_filter_;
    int min_accept_size_;
};

}

#endif //MRVK_PROJECT_MRVKLASERFILTERS_H
