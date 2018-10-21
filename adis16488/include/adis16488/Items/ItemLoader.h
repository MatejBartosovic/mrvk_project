//
// Created by matejko on 21.10.2018.
//

#ifndef PROJECT_ITEMLOADER_H
#define PROJECT_ITEMLOADER_H

#include <yaml-cpp/yaml.h>
#include <adis16488/Items/ItemBase.h>
#include <pluginlib/class_loader.h>
#include <exception>
namespace adis16488{
    namespace Widget{
        class LoaderException : public std::exception {
        public:
            LoaderException(std::string message) : message(message){
            }
            virtual const char* what() const throw()
            {
                return message.c_str();
            }
        private:
            std::string message;
        };
        class AddressNotDefined : public LoaderException {
        public:
            AddressNotDefined(std::string registerName) : LoaderException("Address not define for register "+ registerName){
            }
        };
        class PluginAttributeNotFound : public LoaderException {
            public:
            PluginAttributeNotFound(std::string parrent) : LoaderException("plugin attribute not found in "+ parrent){
                }
        };

        class ItemLoader {
        public:
            ItemLoader();
            boost::shared_ptr<adis16488::Widget::ItemBase> load(std::string registerName, YAML::Node &node, QTreeWidgetItem *rootItem);
            void loadAll(std::string configPath, QTreeWidgetItem *rootItem);
            void loadAll(YAML::Node &node,QTreeWidgetItem *rootItem);
        private:
            pluginlib::ClassLoader<adis16488::Widget::ItemBase> itemLoader;
            std::vector<boost::shared_ptr<adis16488::Widget::ItemBase>> items;
        };
    }
}


#endif //PROJECT_ITEMLOADER_H
