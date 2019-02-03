//
// Created by matejko on 21.10.2018.
//

#include <adis16488/Items/Items.h>
#include <pluginlib/class_list_macros.h>

namespace adis16488{
    namespace Widget{

        // 1 bit item
        size_t BitItem::getDataSize(){
            return 0;
        }
        size_t BitItem::getReadSize(){
            return  0;
        }
        void BitItem::setup(YAML::Node &config){
            ItemBase::setup(config);
            YAML::Node mask = config["mask"];
            if(mask.IsDefined()){
                mask = mask.as<uint8_t>();
            }
        }
        void BitItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,((*(uint8_t*)data) & mask) ? "true" : "false");
        }

        // 8 bit items
        void Uint8HexItem::setValue(void* data){
            char value[5];
            sprintf(value,"0x%02x",*((uint8_t*) data));
            QTreeWidgetItem::setData(3,0,value);
        }
        void Uint8DecimalItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*(uint8_t*)data);
        }

        // 16 bit items
        void Uint16HexItem::setValue(void* data){
            char value[7];
            sprintf(value,"0x%04x",*((uint8_t*) data));
            QTreeWidgetItem::setData(3,0,value);
        }
        void Uint16DecimalItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*(uint16_t*)data);
        }
        void SignedInt16Item::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*(int16_t *)data);
        }

        // 32 bit items
        void FloatItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*((float*) data));
        }

        void Uint32DecimalItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*((uint32_t*) data));
        }

    }
}
// 1 bit
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::BitItem, adis16488::Widget::ItemBase)

// 8 bit
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::Uint8HexItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::Uint8DecimalItem, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyUint8HexItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyUint8DecimalItem, adis16488::Widget::ItemBase)

//16 bit
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::Uint16HexItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::Uint16DecimalItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::SignedInt16Item, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyUint16HexItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyUint16DecimalItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlySignedInt16Item, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::GainOffsetUint16Item, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyGainOffsetUint16Item, adis16488::Widget::ItemBase)

// 32 bit
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::FloatItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::Uint32DecimalItem, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyFloatItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyUint32DecimalItem, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::GainOffsetUint32Item, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::ViewOnlyGainOffsetUint32Item, adis16488::Widget::ItemBase)
