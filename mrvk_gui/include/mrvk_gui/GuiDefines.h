//
// Created by jakub on 14.4.2019.
//

#ifndef PROJECT_GUIDEFINES_H
#define PROJECT_GUIDEFINES_H

#include <QDir>
#include <string>

#define ORIGIN_LATITUDE_PARAM_PATH "/move_base/Planner/origin_latitude"
#define ORIGIN_LONGTITUDE_PARAM_PATH "/move_base/Planner/origin_longitude"

#define TF_WORLD_FRAME "world"
#define TF_BASE_LINK_FRAME "base_link"

#define QLABEL_COLOR_ERROR "QLabel { color : red; }"
#define QLABEL_COLOR_OK "QLabel { color : green; }"
#define QLABEL_COLOR_WARNING "QLabel { color : orange; }"

namespace mrvk_gui {
    namespace topics {
        static const std::string camera = "/mrvk_gui/image";
        static const std::string imu = "/adis16488/imu_data";
        static const std::string odometry = "/odom";
        static const std::string gps = "/gps";
        static const std::string diagnostic = "/diagnostics";
        static const std::string scan = "/scan";
    }
    namespace namespaces {}
}

#endif //PROJECT_GUIDEFINES_H
