//
// Created by matejko on 21.10.2018.
//

#include <adis16488/Items/ItemLoader.h>
#include <pluginlib/class_loader.h>
#include <boost/shared_ptr.hpp>

//TODO prec
#include <iostream>
namespace adis16488{
    namespace Widget{
        ItemLoader::ItemLoader() :itemLoader("adis16488", "adis16488::Widget::ItemBase"){
        }
        boost::shared_ptr<adis16488::Widget::ItemBase> ItemLoader::load(std::string registerName, YAML::Node &node, QTreeWidgetItem *rootItem){
            boost::shared_ptr<adis16488::Widget::ItemBase> newItem;
            try
            {
                YAML::Node pluginNode = node["plugin"];
                if(!pluginNode){
                    throw PluginAttributeNotFound(registerName);
                }
                newItem = itemLoader.createInstance("adis16488::Widget::"+pluginNode.as<std::string>());
                //save shared pointer (prevent to delete object);
                items.push_back(newItem);
                rootItem->addChild(newItem.get());

                //set name
                newItem->name = registerName;
                //set address
                YAML::Node addressNode = node["address"];
                if(!addressNode){
                    throw AddressNotDefined(registerName);
                }
               newItem->registerAddres = addressNode.as<int>();
               YAML::Node readNode = node["read"];
               if(readNode.IsDefined()){
                   newItem->readableRegister = readNode.as<bool>();
               }
               YAML::Node writeNode = node["write"];
               if(writeNode.IsDefined()){
                   newItem->writeableRegister = writeNode.as<bool>();
               }
               newItem->setup();
//               uint8_t a[] = {1,0,0,0};
//               newItem->setValue((void *)a);
            }
            catch(pluginlib::PluginlibException& ex)
            {
                //handle the class failing to load
                ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
            }
            return newItem;
           }
        void ItemLoader::loadAll(std::string configPath, QTreeWidgetItem *rootItem){
            YAML::Node config = YAML::LoadFile(configPath);
            loadAll(config,rootItem);
        }
        void ItemLoader::loadAll(YAML::Node &node,QTreeWidgetItem *rootItem){
            for(YAML::iterator it = node.begin(); it != node.end();it++){
                boost::shared_ptr<adis16488::Widget::ItemBase> newItem =load(it->first.as<std::string>(),it->second,rootItem);
                YAML::Node registerNOde = it->second["Registers"];
                if(registerNOde.IsDefined()){
                    loadAll(registerNOde,newItem.get());
                }
            }
        }
    }
}