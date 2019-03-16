//
// Created by adam on 31.1.2019.
//
#ifndef MRVKLASERFILTERS_H
#define MRVKLASERFILTERS_H

#include <vector>
#include <limits>

#include <boost/circular_buffer.hpp> // TODO continue here

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"

#include "filters/filter_chain.h"

#define IS_INF(val) (std::isinf(val) || std::isnan(val))

namespace mrvk_laser_filters {

    class PointSpeedFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        PointSpeedFilter();

        bool configure() override;

        virtual bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out) override;

    protected:
        double maxVelocity;
        sensor_msgs::LaserScan lastScan;
    };

    class IsolatedPointsFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        IsolatedPointsFilter();

        bool configure() override;

        virtual bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out) override;

    protected:
        int minAcceptSize;
    };

    class TransientPointsFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        TransientPointsFilter();

        bool configure() override;

        virtual bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out) override;

    protected:
        int numPrevFrames;
        boost::circular_buffer<sensor_msgs::LaserScan>* transient_scan_buff_;
    };

}

#endif //MRVKLASERFILTERS_H
