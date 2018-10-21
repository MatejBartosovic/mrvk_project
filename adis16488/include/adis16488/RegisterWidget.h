//
// Created by matejko on 14.10.2018.
//

#ifndef PROJECT_REGISTERWIDGET_H
#define PROJECT_REGISTERWIDGET_H

#include <QTreeWidget>
#include <string>
#include <adis16488/Items/ItemLoader.h>

namespace Adis16488 {
    class RegisterWidget : public QTreeWidget{
    public:
        RegisterWidget(std::string config,QWidget *parrent = NULL);
    private:
        // keep shared pointers to items !!! DO NOT DELETE THIS OBJECT
        adis16488::Widget::ItemLoader loader;
    };
}


#endif //PROJECT_REGISTERWIDGET_H
