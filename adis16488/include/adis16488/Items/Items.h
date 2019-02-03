//
// Created by matejko on 21.10.2018.
//

#ifndef PROJECT_ITEMS_H
#define PROJECT_ITEMS_H

#include <adis16488/Items/ItemBase.h>

namespace adis16488{

    namespace Widget{

        //classes base on size
        class Uint8Item : public ItemBase{
        public:
            size_t getDataSize(){
                return sizeof(uint8_t);
            }
            virtual size_t getReadSize(){
                return  sizeof(uint8_t);
            }
        };

        class Uint16Item : public ItemBase {
        public:
            size_t getDataSize(){
                return sizeof(uint16_t);
            }
            virtual size_t getReadSize(){
                return  sizeof(uint16_t);
            }
        };
        class Uint32Item : public ItemBase{
        public:
            size_t getDataSize(){
                return sizeof(uint32_t);
            }
            virtual size_t getReadSize(){
                return  sizeof(uint32_t);
            }
        };

        // GainOffset classes

        //GainOffset(Uint8Item,int8_t);
        GainOffset(Uint16Item,int16_t);
        //GainOffset(Uint32Item,int32_t)

        // viwew only classes
        ViewOnly(Uint8Item);
        ViewOnly(Uint16Item);
        ViewOnly(Uint32Item);
        //ViewOnly(GainOffsetUint8Item);
        ViewOnly(GainOffsetUint16Item);
        //ViewOnly(GainOffsetUint32Item);


        // 1 bit item
        class BitItem : public ItemBase {
        public:
            size_t getDataSize();
            size_t getReadSize();
            virtual void setup(YAML::Node &config);
            virtual void setValue(void* data);
        private:
            uint8_t mask;
        };


        // 8 bit items
        class Uint8HexItem : public Uint8Item{
        public:
            void setValue(void* data);
        };

        class Uint8DecimalItem : public Uint8Item{
        public:
            void setValue(void* data);
        };

        ViewOnly(Uint8HexItem);
        ViewOnly(Uint8DecimalItem);

        // 16 bit items
        class Uint16HexItem : public Uint16Item{
        public:
            void setValue(void* data);
        };

        class Uint16DecimalItem : public Uint16Item{
        public:
            void setValue(void* data);
        };

        class SignedInt16Item : public Uint16Item{
        public:
            void setValue(void* data);
        };

        ViewOnly(Uint16HexItem);
        ViewOnly(Uint16DecimalItem);
        ViewOnly(SignedInt16Item);


        // 32 bit items
        class FloatItem : public Uint32Item {
        public:
            void setValue(void* data);
        };

        class Uint32DecimalItem : public Uint32Item{
        public:
            void setValue(void* data);
        };
        ViewOnly(FloatItem);
        ViewOnly(Uint32DecimalItem);
        GainOffset(Uint32Item,int32_t);
        ViewOnly(GainOffsetUint32Item);


    }
}


#endif //PROJECT_ITEMS_H
