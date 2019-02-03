//
// Created by matejko on 3.2.2019.
//

#ifndef PROJECT_GAINOFFSETITEM_H
#define PROJECT_GAINOFFSETITEM_H

#include <yaml-cpp/yaml.h>

class GainOffset{
public:
    void setup(YAML::Node &node){
        YAML::Node offsetNode = node["offset"];
        YAML::Node gainNode = node["gain"];
        if(!(offsetNode.IsDefined() && gainNode.IsDefined())){
            throw std::runtime_error("gain or offset parameter missing"); //TODO own exception
        }
        offset = offsetNode.as<double>();
        gain = gainNode.as<double>();
    }
    template<typename T>
    T compute(T input){
        return (input * gain) + offset;
    }

protected:
    double gain;
    double offset;
};

#define GainOffset(classname,dataType) class GainOffset##classname : public classname\
{ \
    public: \
    virtual void setup(YAML::Node &node){ \
        gainOffset.setup(node); \
        ItemBase::setup(node); \
    } \
    void setValue(void* data){ \
        QTreeWidgetItem::setData(3,0,gainOffset.compute(*(dataType*)data)); \
    } \
    protected: \
        GainOffset gainOffset; \
};

#endif //PROJECT_GAINOFFSETITEM_H
