//
// Created by matejko on 23.3.2019.
//

#ifndef PROJECT_IMUBUTTONS_H
#define PROJECT_IMUBUTTONS_H

#include <ros/service_client.h>

namespace mrvk_gui {
    class ImuButtons {
    public:
        ImuButtons();

        void calibrate();

        void resetAdis();

        void resetGyro();

        void reset();
    private:
        ros::ServiceClient calibrateService;
        ros::ServiceClient resetAdisService;
        ros::ServiceClient resetGyroService;
    };
}

#endif //PROJECT_IMUBUTTONS_H
