//
// Created by matejko on 19.3.2019.
//

#ifndef PROJECT_TOPICFREQUENCY_H
#define PROJECT_TOPICFREQUENCY_H

#include <mrvk_gui/Subscriber.h>
#include <ros/node_handle.h>
#include <mutex>

namespace mrvk_gui {
    template <typename T>
    class TopicFrequencySubscriber {
    protected:
        typedef boost::shared_ptr<T const> TConstPtr;
    public:
        TopicFrequencySubscriber(std::string topic, ros::NodeHandle& nh) : counter(0), oldTime(ros::Time::now()){
            boost::bind(&TopicFrequencySubscriber::calback,this,_1);
            sub = nh.subscribe<T>(topic,5,boost::bind(&TopicFrequencySubscriber<T>::calback,this,_1));
        }

        double getFrequency() {
            ros::Time newTime = ros::Time::now();
            std::lock_guard<std::mutex> l(mutex);
            double frequency = counter / (newTime - oldTime).toSec();
            oldTime = newTime;
            counter = 0;
            return frequency;
        }

    private:
        void calback(boost::shared_ptr<T const> msg){
            std::lock_guard<std::mutex> lock(mutex);
            counter++;
        }

        std::mutex mutex;
        ros::Subscriber sub;
        ros::Time oldTime;
        uint32_t counter;
    };
}

#endif //PROJECT_TOPICFREQUENCY_H
