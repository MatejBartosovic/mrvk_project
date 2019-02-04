//
// Created by adam on 31.1.2019.
//

#ifndef MRVK_PROJECT_MRVKLASERFILTERS_H
#define MRVK_PROJECT_MRVKLASERFILTERS_H

#include <vector>
#include <limits>

#include <boost/circular_buffer.hpp> // TODO continue here

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include "filters/filter_chain.h"

namespace mrvk_laser_filters {

#define IS_INF(val) (std::isinf(val) || std::isnan(val))

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

    /**
     *
     * @param scan_in
     * @param scan_out
     * @return
     */
    bool updateTransientPointsFilter(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);


private:
    // Common
    ros::Time t_last_message_;
    sensor_msgs::LaserScan prev_scan_;

    // Isolated Points filter
    bool enable_isolated_points_filter_;
    int min_accept_size_;

    // Transient Points filter
    bool enable_transient_points_filter_;
    int num_prev_frames_;
    boost::circular_buffer<sensor_msgs::LaserScan>* transient_scan_buff_;

    // Utils
    void printScanHistogram(const sensor_msgs::LaserScan& scan_in);
    void printScanDiffHistogram(const sensor_msgs::LaserScan& scan_in, const sensor_msgs::LaserScan& scan_out);
};

}

#endif //MRVK_PROJECT_MRVKLASERFILTERS_H
