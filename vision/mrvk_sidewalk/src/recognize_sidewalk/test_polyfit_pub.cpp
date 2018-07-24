//
// Created by smadas on 19.7.2018.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "polyfit2.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_polyfit_pub");

    ros::NodeHandle n;

    ros::Publisher polyfit_pub = n.advertise<std_msgs::Float64>("polyfit", 1);

    ros::Rate loop_rate(10);

    double const x[30] = {

    0.0703,
    0.1141,
    0.1878,
    0.1901,
    0.2270,
    0.2592,
    0.2984,
    0.3399,
    0.3652,
    0.3952,
    0.4505,
    0.4896,
    0.5311,
    0.5541,
    0.5841,
    0.6187,
    0.6440,
    0.6901,
    0.7270,
    0.7615,
    0.7892,
    0.8053,
    0.8283,
    0.8514,
    0.8882,
    0.9021,
    0.9021,
    0.9182,
    0.9343,
    0.9435
    };


    double const y[30] = {

    0.2551,
    0.3338,
    0.3630,
    0.3630,
    0.2843,
    0.2172,
    0.2376,
    0.3076,
    0.3601,
    0.4300,
    0.4913,
    0.5379,
    0.4883,
    0.4359,
    0.3950,
    0.3367,
    0.2697,
    0.2143,
    0.2318,
    0.2872,
    0.3338,
    0.3805,
    0.4300,
    0.4942,
    0.5437,
    0.6341,
    0.6953,
    0.7799,
    0.8178,
    0.8528,
    };
    double p[3];
    polyfit2(x, y, (unsigned int)30, (unsigned int)3, p);

    std::cout << "p " << p << std::endl;

    int count = 0;
    while (ros::ok())
    {

        std_msgs::Float64 msg;

        //std::stringstream ss;
        //ss << "hello world " << count;
        msg.data = count;

        //ROS_INFO("%s", msg.data.c_str());

        polyfit_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}