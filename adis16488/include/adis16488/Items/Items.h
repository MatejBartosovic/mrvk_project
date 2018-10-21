//
// Created by matejko on 21.10.2018.
//

#ifndef PROJECT_ITEMS_H
#define PROJECT_ITEMS_H

#include <adis16488/Items/ItemBase.h>

namespace adis16488{
    namespace Widget{
        class HexItem : public ItemBase {
        public:
            HexItem();
            void setValue(void* data);
            size_t getSize();
        };
        class FloatItem : public ItemBase {
        public:
            FloatItem();
            void setValue(void* data);
            size_t getSize();
        };
    }
}


#endif //PROJECT_ITEMS_H
