//
// Created by matejko on 14.10.2018.
//

#include "adis16488/RegisterWidget.h"

namespace Adis16488 {

    RegisterWidget::RegisterWidget(std::string config,QWidget *parrent) : QTreeWidget(parrent){
        QSizePolicy policy(QSizePolicy::Policy::Preferred,QSizePolicy::Policy::Preferred);
        this->setSizePolicy(policy);
        QStringList headerList = {"Name","Address","Flags","Value","Description"};
        this->setHeaderItem(new QTreeWidgetItem(headerList));
        loader.loadAll(config,this->invisibleRootItem());
    }
}
