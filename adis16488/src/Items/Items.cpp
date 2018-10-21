//
// Created by matejko on 21.10.2018.
//

#include <adis16488/Items/Items.h>
#include <pluginlib/class_list_macros.h>

namespace adis16488{
    namespace Widget{
        HexItem::HexItem(){

        }
        void HexItem::setValue(void* data){
            char value[5];
            sprintf(value,"0x%02x",*((uint8_t*) data));
            this->setData(3,0,value);
        }

        size_t HexItem::getSize(){
            return sizeof(uint8_t);
        }

        FloatItem::FloatItem(){

        }
        void FloatItem::setValue(void* data){
            float value = *((float*) data);
            this->setData(3,0,std::to_string(value).c_str());
        }

        size_t FloatItem::getSize(){
            sizeof(float);
        }
    }
}
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::FloatItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::HexItem, adis16488::Widget::ItemBase)
