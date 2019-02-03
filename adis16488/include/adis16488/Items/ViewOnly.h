//
// Created by matejko on 3.2.2019.
//

#ifndef PROJECT_VIEWONLY_H
#define PROJECT_VIEWONLY_H

#define ViewOnly(classname) class ViewOnly##classname : public classname\
{ \
    public: \
    size_t getReadSize(){ \
        return  0; \
    } \
};

#endif //PROJECT_VIEWONLY_H
