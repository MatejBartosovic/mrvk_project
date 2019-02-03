//
// Created by matejko on 21.10.2018.
//

#ifndef PROJECT_ITEMBASE_H
#define PROJECT_ITEMBASE_H

#include <QTreeWidgetItem>
#include <sstream>
#include <yaml-cpp/yaml.h>

class ItemLoader;

namespace adis16488{
    namespace Widget{
        class ItemBase : public QTreeWidgetItem{
        public:
            ItemBase() : readableRegister(false), writeableRegister(false), name("Undefined"),registerAddres(0x00){

            }
            virtual void setValue(void* data) = 0;
            virtual size_t getReadSize() = 0;
            virtual size_t getDataSize() = 0;
            virtual void setup(YAML::Node &config){
                this->setData(0,0,QString(name.c_str()));
                char tmp[5];
                sprintf(tmp,"0x%02x",registerAddres);
                this->setData(1,0,tmp);
                std::string flags;
                if(isReadable()){
                    flags.push_back('R');
                }
                if(isWritable()){
                    flags.push_back('W');
                }
                this->setData(2,0,flags.c_str());
                this->setData(3,0,"Nan");
                this->setData(4,0,description.c_str());
            }
            bool isWritable(){
                return writeableRegister;
            }
            bool isReadable(){
                return writeableRegister;
            }
            std::string getName(){
                return name;
            }
            uint8_t getAddress(){
                return registerAddres;
            }
            friend class ItemLoader;
        private:
            std::string name;
            uint8_t registerAddres;
            bool readableRegister;
            bool writeableRegister;
            std::string description;
        };
    }
}

#endif //PROJECT_ITEMBASE_H
