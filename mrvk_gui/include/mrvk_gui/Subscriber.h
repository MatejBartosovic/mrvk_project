//
// Created by matejko on 10.3.2019.
//

#ifndef PROJECT_SUBSCRIBER_H
#define PROJECT_SUBSCRIBER_H

#include <ros/ros.h>
#include <mutex>
#include <std_msgs/String.h>

namespace mrvk_gui{
    template <class T>
    class Subscriber {
    protected:
        typedef boost::shared_ptr<T const> TConstPtr;
    public:
        Subscriber(std::string topic, ros::NodeHandle& nh){
            boost::bind(&Subscriber::callback,this,_1);
            subscriber = nh.subscribe<T>(topic,5,boost::bind(&Subscriber<T>::callback,this,_1));
        }

        const T& getData(){
            std::lock_guard<std::mutex> lock(mutex);
            return msg;
        }

    private:
        void callback(TConstPtr msg)
        {
            if(!msgValidityCallback(msg)){
                return;
            }
            std::lock_guard<std::mutex> lock(mutex);
            this->msg = *msg;
        }

        virtual bool msgValidityCallback(TConstPtr msg) {
            return true;
        };

        std::mutex mutex;
        T msg;
        ros::Subscriber subscriber;
    };
}

#endif //PROJECT_SUBSCRIBER_H
