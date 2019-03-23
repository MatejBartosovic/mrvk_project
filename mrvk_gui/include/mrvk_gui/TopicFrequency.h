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
        TopicFrequencySubscriber(std::string topic, ros::NodeHandle& nh){
            boost::bind(&TopicFrequencySubscriber::calback,this,_1);
            sub = nh.subscribe<T>(topic,5,boost::bind(&TopicFrequencySubscriber<T>::calback,this,_1));
        }

        double getFrequency() {
            std::lock_guard<std::mutex> l(mutex);
            return frequency;
        }

    private:
        void calback(boost::shared_ptr<T const> msg){
            std::lock_guard<std::mutex> l(mutex);

            newTime = ros::Time::now();
            frequency = 1 / (newTime - oldTime).toSec();
            //frequency = round( frequency * 1000.0 ) / 1000.0;
            oldTime = newTime;
        }

        std::mutex mutex;
        double frequency = 0;
        ros::Subscriber sub;
        ros::Time oldTime;
        ros::Time newTime;
    };
}

#endif //PROJECT_TOPICFREQUENCY_H
