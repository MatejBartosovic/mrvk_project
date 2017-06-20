//
// Created by michal on 12.6.2017.
//

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

template<typename T> class CovarianceCalculator{

public:
    CovarianceCalculator() : counter(0), buffer(0), realValue(0){}

    void setRealValue(T data){
        realValue = data;
        clearBuffer();
    }

    double addMeassure(const T& data){

        T value = pow(realValue - data, 2.0);
        buffer += value;
        counter++;
        return sqrt(value);
    }

    T computeCovariance() {

     return  sqrt(buffer/counter);
    }

    void clearBuffer(){
        counter = 0;
        buffer = 0;
    }

private:
    T buffer;
    T realValue;
    int counter;

};




CovarianceCalculator <double>imu_x;
CovarianceCalculator <double>imu_y;
CovarianceCalculator <double>imu_z;

void callback(const sensor_msgs::Imu& data) {

    static int counter = 0;

    if (counter < 1000) {
        imu_x.addMeassure(data.angular_velocity.x);
        imu_y.addMeassure(data.angular_velocity.y);
        ROS_INFO("%f", imu_z.addMeassure(data.angular_velocity.z));
    } else {

        ROS_WARN("covariance x: %f y: %f z: %f", imu_x.computeCovariance(), imu_y.computeCovariance(), imu_z.computeCovariance());
        imu_x.clearBuffer();
        imu_z.clearBuffer();
        imu_y.clearBuffer();
        counter  = 0;
    }

}


bool computeCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

    ROS_WARN("covariance x: %f y: %f z: %f", imu_x.computeCovariance(), imu_y.computeCovariance(), imu_z.computeCovariance());
    imu_x.clearBuffer();
    imu_z.clearBuffer();
    imu_y.clearBuffer();
    return true;
}

int main (int argc, char **argv){

    ros::init(argc, argv, "covariance_calculator");
    ros::NodeHandle node;

    ros::Subscriber imu_sub = node.subscribe("adis16350/imu_data", 10, callback);

    ros::ServiceServer compute_service = node.advertiseService("compute", &computeCallback);


    ros::spin();

}