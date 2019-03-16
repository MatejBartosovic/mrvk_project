//
// Created by adam on 31.1.2019.
//

#include <mrvk_laser_filter/MrvkLaserFilters.h>
#include <XmlRpcException.h> //TODO REMOVE

#include <pluginlib/class_list_macros.h>

namespace mrvk_laser_filters {

    PointSpeedFilter::PointSpeedFilter() : maxVelocity(3){

    }

    bool PointSpeedFilter::configure(){
        getParam("point_speed_max",maxVelocity);

    }

    bool PointSpeedFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out){
        data_out = data_in;
        if(data_in.ranges.size() != lastScan.ranges.size()){
            lastScan = data_in;
            ROS_WARN("New scan has different size than last scan.");
            return true;
        }

        double dt = ros::Duration(data_in.header.stamp - lastScan.header.stamp).toSec();
//      std::cout << dt << std::endl;
        if(dt<0.01)
            dt = 0.01;
        else if(dt>1.0)
            dt = 1.0;

        for (int i = 0; i < data_in.ranges.size(); ++i) {
            double dp = std::abs(data_in.ranges.at(i) - lastScan.ranges.at(i));
            double v = std::abs(dp/dt);
//        std::cout << v << " ";
            if(v > maxVelocity){
                data_out.ranges.at(i) = std::numeric_limits<float>::infinity();
            }
        }
        lastScan = data_in;
//    std::cout << std::endl;
    }

    IsolatedPointsFilter::IsolatedPointsFilter() : minAcceptSize(3){

    }

    bool IsolatedPointsFilter::configure(){
        getParam("min_accept_size",minAcceptSize);
    }

    bool IsolatedPointsFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out){
        data_out = data_in;
        int start_i = 0;
        int end_i = 0;
        int count = 0;
        bool is_region = false;

        for(int i = 0; i<data_out.ranges.size(); i++){
            float dist = data_out.ranges.at(i);
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
                    if (count < minAcceptSize) {
                        for(int j = start_i; j<end_i; j++) {
                            data_out.ranges.at(j) = INFINITY;
                        }
                    }
                    count = 0;
                }
            }

        }
        return true;
    }

    TransientPointsFilter::TransientPointsFilter() : numPrevFrames(2){

    }

    bool TransientPointsFilter::configure(){
        getParam("min_accept_size",numPrevFrames);
    }

    bool TransientPointsFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out){
        data_out = data_in;
        if(!transient_scan_buff_->empty() && transient_scan_buff_->back().ranges.size() != data_in.ranges.size()){
            ROS_WARN("Scan size changed from %ld to %ld, rebuilding buffer", transient_scan_buff_->back().ranges.size(), data_in.ranges.size());
            transient_scan_buff_->clear();
        }

        transient_scan_buff_->push_back(data_in);
        if(!transient_scan_buff_->full()){
            ROS_DEBUG("Filling scan buff %ld", transient_scan_buff_->size());
            return true;
        }

        int scan_size = data_in.ranges.size();
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
                data_out.ranges.at(i) = std::numeric_limits<float>::infinity();
        }

        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(mrvk_laser_filters::PointSpeedFilter, filters::FilterBase<sensor_msgs::LaserScan>);
PLUGINLIB_EXPORT_CLASS(mrvk_laser_filters::IsolatedPointsFilter, filters::FilterBase<sensor_msgs::LaserScan>);
PLUGINLIB_EXPORT_CLASS(mrvk_laser_filters::TransientPointsFilter, filters::FilterBase<sensor_msgs::LaserScan>);