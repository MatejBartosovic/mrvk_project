//
// Created by adam on 31.1.2019.
//

#include "mrvk_laser_filters.h"

#include <pluginlib/class_list_macros.h>
#include <xmlrpcpp/XmlRpcException.h>

PLUGINLIB_EXPORT_CLASS(mrvk_laser_filters::MrvkLaserFilters, filters::FilterBase<sensor_msgs::LaserScan>);

namespace mrvk_laser_filters {

MrvkLaserFilters::MrvkLaserFilters():
        t_last_message_(0), \
        enable_isolated_points_filter_(false), min_accept_size_(1), \
        enable_transient_points_filter_(false), num_prev_frames_(1), transient_scan_buff_(NULL)
{
    //
}

MrvkLaserFilters::~MrvkLaserFilters() {
    if(transient_scan_buff_)
        delete transient_scan_buff_;
}

bool MrvkLaserFilters::configure() {

    XmlRpc::XmlRpcValue ip_params, tp_params;
    if(getParam("isolated_points_filter", ip_params)) {
        for (auto it = ip_params.begin() ;it != ip_params.end();it++) {
            std::cout << "param " << it->first << " has value " << ip_params[it->first]  << " type " << ip_params[it->first].getType() << std::endl;
        }
        try {
            if (ip_params.hasMember("enabled")) {
                enable_isolated_points_filter_ = ip_params["enabled"];
                min_accept_size_ = ip_params["min_accept_size"];
            }
        }
        catch (XmlRpc::XmlRpcException exc) {
            ROS_ERROR("IsolatedPointsFilter wrong or missing parameters!");
            std::cout << exc.getMessage() << std::endl;
        }
    }

    if(getParam("transient_points_filter", tp_params)) {
        for (auto it = tp_params.begin() ;it != tp_params.end();it++) {
            std::cout << "param " << it->first << " has value " << ip_params[it->first]  << " type " << ip_params[it->first].getType() << std::endl;
        }
        try {
            if (tp_params.hasMember("enabled")) {
                enable_transient_points_filter_ = tp_params["enabled"];
                num_prev_frames_ = tp_params["num_prev_frames"];
            }
        }
        catch (XmlRpc::XmlRpcException exc) {
            ROS_ERROR("TransientPointsFilter wrong or missing parameters!");
            std::cout << exc.getMessage() << std::endl;
        }

        transient_scan_buff_ = new boost::circular_buffer<sensor_msgs::LaserScan>(num_prev_frames_);
    }

//    auto it = ip_params.find
//    ros::NodeHandle nh("~");
//    ROS_WARN("%s", nh.getNamespace().c_str());
//
//    nh.getParam("scan_filter_chain/mrvk_laser_filters/ip_params/isolated_points_filter/enabled", enable_isolated_points_filter_);
//    nh.getParam("isolated_points_filter/min_accept_size", min_accept_size_);

    ROS_WARN("IsolatedPointFilter enabled = %d, min_accept_size = %d", enable_isolated_points_filter_, min_accept_size_);
    ROS_WARN("TransientPointFilter enabled = %d, num_prev_frames = %d", enable_transient_points_filter_, num_prev_frames_);
    if(!enable_transient_points_filter_ && !enable_isolated_points_filter_)
        ROS_FATAL("NO LASER FILTER ENABLED!!!");
}

/** \brief Update the filter and get the response
 * \param scan_in The new scan to filter
 * \param scan_out The filtered scan
 */
bool MrvkLaserFilters::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
//    printScanHistogram(scan_in);

    if(enable_isolated_points_filter_) {
        updateIsolatedPointsFilter(scan_in, scan_out);
    }

    if(enable_transient_points_filter_) {
        updateTransientPointsFilter(scan_in, scan_out);
    }

//    printScanHistogram(scan_out);
//    std::cout << std::endl;
    printScanDiffHistogram(scan_in, scan_out);
    prev_scan_ = scan_in;
    t_last_message_ = scan_in.header.stamp;

    return true;
}

bool MrvkLaserFilters::updateIsolatedPointsFilter(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
    scan_out = scan_in;
    int start_i = 0;
    int end_i = 0;
    int count = 0;
    bool is_region = false;

    for(int i = 0; i<scan_out.ranges.size(); i++){
        float dist = scan_out.ranges.at(i);
        if(!std::isinf(dist) && !std::isnan(dist)){
            count += 1;
            if (!is_region) {
                is_region = true;
                start_i = i;
            }
        }
        else {
            if(is_region) {
                is_region = false;
                end_i = i;
                if (count < min_accept_size_) {
                    for(int j = start_i; j<end_i; j++) {
                        scan_out.ranges.at(j) = INFINITY;
                    }
                }
                count = 0;
            }
        }

    }
    return true;
}

bool MrvkLaserFilters::updateTransientPointsFilter(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
    scan_out = scan_in;
    if(!transient_scan_buff_->empty() && transient_scan_buff_->back().ranges.size() != scan_in.ranges.size()){
        ROS_WARN("Scan size changed from %ld to %ld, rebuilding buffer", transient_scan_buff_->back().ranges.size(), scan_in.ranges.size());
        transient_scan_buff_->clear();
    }

    transient_scan_buff_->push_back(scan_in);
    if(!transient_scan_buff_->full()){
        ROS_DEBUG("Filling scan buff %ld", transient_scan_buff_->size());
        return true;
    }

    int scan_size = scan_in.ranges.size();
    std::vector<bool> validity_mask(scan_size, true); // Starts True for all scan elements

    for (int ib = 0; ib < transient_scan_buff_->size(); ++ib) {
        for (int is = 0; is < scan_size; ++is) {
            // Set mask element to False if the element is invalid (is infinity or NaN)
            float dist = transient_scan_buff_->at(ib).ranges.at(is);
            validity_mask[is] = validity_mask[is] & !IS_INF(dist);
        }
    }

    for (int i = 0; i < scan_size; ++i) {
        if(!validity_mask[i])
            scan_out.ranges.at(i) = std::numeric_limits<float>::infinity();
    }

    return true;
}

void MrvkLaserFilters::printScanHistogram(const sensor_msgs::LaserScan& scan_in) {
    std::cout << scan_in.header.stamp << " ";
    for(int i = 0; i<scan_in.ranges.size(); i++){
        float dist = scan_in.ranges.at(i);
        if(std::isinf(dist) || std::isnan(dist)){
            std::cout << "_";
        }
        else {
            std::cout << "#";
        }
    }
    std::cout << std::endl;
}

void MrvkLaserFilters::printScanDiffHistogram(const sensor_msgs::LaserScan& scan_in, const sensor_msgs::LaserScan& scan_out) {
    std::cout << scan_in.header.stamp << " ";
    for(int i = 0; i<scan_in.ranges.size(); i++){
        float dist_in = scan_in.ranges.at(i);
        float dist_out = scan_out.ranges.at(i);
        if(!IS_INF(dist_in) && IS_INF(dist_out)) {
            std::cout << "x";
        }
        else if(IS_INF(dist_in)){
            std::cout << "_";
        }
        else {
            std::cout << "#";
        }
    }
    std::cout << std::endl;
}

}

/*
 * TODO:
 * */
