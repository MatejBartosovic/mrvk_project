//
// Created by matejko on 19.3.2019.
//

#ifndef PROJECT_TOPICFREQUENCY_H
#define PROJECT_TOPICFREQUENCY_H

#include <mrvk_gui/Subscriber.h>
#include <ros/node_handle.h>

namespace mrvk_gui {
    template <typename T>
    class TopicFrequencySubscriber : public Subscriber<T> {
    public:
        TopicFrequencySubscriber(std::string topic, ros::NodeHandle& nh) : mrvk_gui::Subscriber<T>(topic, nh) {

        }

        double getFrequency() {
            return 0.5;
        }

        bool msgValidityCallback(boost::shared_ptr<T const> msg) override {
            return true;
        };
    };
}

#endif //PROJECT_TOPICFREQUENCY_H
