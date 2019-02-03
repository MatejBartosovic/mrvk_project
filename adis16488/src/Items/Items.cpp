//
// Created by matejko on 21.10.2018.
//

#include <adis16488/Items/Items.h>
#include <pluginlib/class_list_macros.h>

namespace adis16488{
    namespace Widget{
        GainOffsetItem::GainOffsetItem() : gain(0.0), offset(0.0){

        }
        void GainOffsetItem::setup(YAML::Node &node){
            YAML::Node offsetNode = node["offset"];
            YAML::Node gainNode = node["gain"];
            if(!(offsetNode.IsDefined() && gainNode.IsDefined())){
                throw std::runtime_error("gain or offset parameter missing"); //TODO own exception
            }
            offset = offsetNode.as<double>();
            gain = gainNode.as<double>();
            ItemBase::setup(node);
        }
        void HexByteItem::setValue(void* data){
            char value[5];
            sprintf(value,"0x%02x",*((uint8_t*) data));
            QTreeWidgetItem::setData(3,0,value);
        }
        size_t HexByteItem::getReadSize(){

        }
        size_t HexByteItem::getDataSize(){

        }
        void DecimalByteItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*(uint8_t*)data);
        }
        void BitDispayItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,(*(uint8_t*)data) & ItemBase::getAddress() ? "true" : "false");
        }
        size_t BitDispayItem::getDataSize(){
            return 0;
        }
        size_t BitDispayItem::getReadSize(){
            return 0;
        }
        size_t DisplayOnlyHexByteItem::getReadSize(){
            return 0;
        }
        size_t DisplayOnlyDecimalByteItem::getReadSize(){
            return 0;
        }
        void DecimalByteGainOffsetItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,offset + (gain * (*(int8_t*)data)));
        }
        size_t DisplayOnlyDecimalByteGainOffsetItem::getReadSize(){
            return 0;
        }
        size_t DecimalByteGainOffsetItem::getReadSize(){
            return sizeof(uint8_t);
        }
        size_t DecimalByteGainOffsetItem::getDataSize(){
            return sizeof(uint8_t);
        }
        void HexUint16Item::setValue(void* data){
            char value[7];
            sprintf(value,"0x%04x",*((uint16_t*) data));
            QTreeWidgetItem::setData(3,0,value);
        }
        size_t HexUint16Item::getDataSize(){
            return sizeof(uint16_t);
        }
        size_t HexUint16Item::getReadSize(){
            return sizeof(uint16_t);
        }
        void DecimalUint16Item::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*(uint16_t*)data);
        }
        void SignedInt16Item::setValue(void* data){
            int16_t  value = *(uint16_t*)data;
            QTreeWidgetItem::setData(3,0,value);
        }
        void Int16GainOffsetItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,offset + (gain * (*(int16_t*)data)));
        }
        size_t Int16GainOffsetItem::getDataSize(){
            return sizeof(uint16_t);
        }
        size_t Int16GainOffsetItem::getReadSize(){
            return sizeof(uint16_t);
        }
        void FloatItem::setValue(void* data){
            QTreeWidgetItem::setData(3,0,*((float*) data));
        }
        size_t FloatItem::getDataSize(){
            return sizeof(uint32_t);
        }
        size_t FloatItem::getReadSize(){
            return sizeof(uint32_t);
        }
    }
}
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::HexByteItem, adis16488::Widget::ItemBase)                         //uint8_t register print ax hex number
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::DecimalByteItem, adis16488::Widget::ItemBase)                     //uint8_t register print as decimal number
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::DisplayOnlyHexByteItem, adis16488::Widget::ItemBase)              //uint8_t register print ax hex number (sub print of larger register)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::DisplayOnlyDecimalByteItem, adis16488::Widget::ItemBase)          //uint8_t register print ax hex number (sub print of larger register)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::DecimalByteGainOffsetItem, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::DisplayOnlyDecimalByteGainOffsetItem, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::BitDispayItem, adis16488::Widget::ItemBase)                       //bit in register (display bit value based on address (used as mask) - typical parrent BinaryItem or HexItem)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::HexUint16Item, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::DecimalUint16Item, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::SignedInt16Item, adis16488::Widget::ItemBase)
PLUGINLIB_EXPORT_CLASS(adis16488::Widget::Int16GainOffsetItem, adis16488::Widget::ItemBase)

PLUGINLIB_EXPORT_CLASS(adis16488::Widget::FloatItem, adis16488::Widget::ItemBase)