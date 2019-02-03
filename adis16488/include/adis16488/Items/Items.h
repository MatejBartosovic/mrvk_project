//
// Created by matejko on 21.10.2018.
//

#ifndef PROJECT_ITEMS_H
#define PROJECT_ITEMS_H

#include <adis16488/Items/ItemBase.h>

namespace adis16488{
    namespace Widget{
        class GainOffsetItem : public ItemBase{
        public:
            GainOffsetItem();
            virtual void setup(YAML::Node &node);
        protected:
            double offset;
            double gain;
        };
        class HexByteItem : public ItemBase {
        public:
            virtual void setValue(void* data);
            virtual size_t getReadSize();
            size_t getDataSize();
        };
        class DecimalByteItem : public HexByteItem {
        public:
            virtual void setValue(void* data);
        };
        class DisplayOnlyHexByteItem : public HexByteItem {
        public:
            size_t getReadSize();
        };
        class DisplayOnlyDecimalByteItem : public DecimalByteItem {
        public:
            size_t getReadSize();
        };
        class DecimalByteGainOffsetItem : public GainOffsetItem {
        public:
            virtual void setValue(void* data);
            virtual size_t getReadSize();
            size_t getDataSize();
        };
        class DisplayOnlyDecimalByteGainOffsetItem : public DecimalByteGainOffsetItem {
        public:
            size_t getReadSize();
        };
        class BitDispayItem : public ItemBase{
        public:
            virtual void setValue(void* data);
            size_t getDataSize();
            size_t getReadSize();
        };
        class HexUint16Item : public ItemBase {
        public:
            void setValue(void* data);
            size_t getDataSize();
            size_t getReadSize();
        };
        class DecimalUint16Item : public HexUint16Item {
        public:
            void setValue(void* data);
        };
        class SignedInt16Item : public HexUint16Item {
        public:
            void setValue(void* data);
        };
        class Int16GainOffsetItem : public GainOffsetItem{
        public:
            void setValue(void* data);
            size_t getDataSize();
            size_t getReadSize();
        };
        class FloatItem : public ItemBase {
        public:
            void setValue(void* data);
            size_t getDataSize();
            size_t getReadSize();
        };
//        class Int16GainOffsetItem : public ReadUint16, DataUint16, GainOffsetItem {
//        public:
//            Int16GainOffsetItem();
//            void setValue(void* data);
//        };
//        class DecimalByteItem : public ReadUint8, DataUint8 {
//        public:
//            DecimalByteItem();
//            void setValue(void* data);
//        };
//        class DecimalByteGainOffsetItem : public DecimalByteItem, GainOffsetItem {
//        public:
//            DecimalByteGainOffsetItem();
//            void setValue(void* data);
//        };
//        class ByteDispayItem : public ReadNothingItem, DataUint8 {
//        public:
//            ByteDispayItem();
//            void setValue(void* data);
//            virtual size_t getDataSize();
//        };

    }
}


#endif //PROJECT_ITEMS_H
