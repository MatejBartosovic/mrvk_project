//
// Created by matejko on 23.3.2019.
//

#include <mrvk_gui/ImuButtons.h>
#include <mrvk_gui/ButtonException.h>
#include <std_srvs/Trigger.h>
#include <ros/node_handle.h>

namespace mrvk_gui {
    ImuButtons::ImuButtons(){
        ros::NodeHandle n("adis16488");
        calibrateService =  n.serviceClient<std_srvs::Trigger>("calib_adis_srv");
        resetAdisService =  n.serviceClient<std_srvs::Trigger>("reset_adis_srv");
        resetGyroService =  n.serviceClient<std_srvs::Trigger>("reset_gyro_srv");

    }

    void ImuButtons::calibrate(){
        std_srvs::Trigger msg;
        callService(calibrateService,msg);
    }

    void ImuButtons::resetAdis(){
        std_srvs::Trigger msg;
        callService(resetAdisService,msg);
    }

    void ImuButtons::resetGyro(){
        std_srvs::Trigger msg;
        callService(resetGyroService,msg);
    }

    void ImuButtons::reset(){
        resetAdis();
        resetGyro();
    }
}