//
// Created by matejko on 21.10.2018.
//

#ifndef PROJECT_ITEMBASE_H
#define PROJECT_ITEMBASE_H

#include <QTreeWidgetItem>
#include <sstream>

class ItemLoader;

namespace adis16488{
    namespace Widget{
        class ItemBase : public QTreeWidgetItem{
        public:
            ItemBase() : readableRegister(false), writeableRegister(false), name("Undefined"),registerAddres(0x00){

            }
            virtual void setValue(void* data) = 0;
            virtual size_t getSize() = 0;
            virtual bool isReadable(){
                return readableRegister;
            }
            virtual bool isWritable(){
                return writeableRegister;
            }
            std::string getName(){
                return name;
            }
            virtual void setup(){
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
            }
            friend class ItemLoader;
        private:
            std::string name;
            uint8_t registerAddres;
            bool readableRegister;
            bool writeableRegister;
        };
    }
}

#endif //PROJECT_ITEMBASE_H
