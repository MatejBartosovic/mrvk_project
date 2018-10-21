//
// Created by matejko on 14.10.2018.
//

#include <QApplication>
#include <QMainWindow>
#include <adis16488/RegisterWidget.h>
#include <QVBoxLayout>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QMainWindow mw;
    Adis16488::RegisterWidget registerWidget("/home/matejko/mrvk_ws/src/mrvk_project/adis16488/config/adis16488.yaml");
    mw.setCentralWidget(&registerWidget);
//    QStringList list = {"aaa","bbb","ccc"};
//    QTreeWidgetItem item(list);
//    QTreeWidgetItem item2(list);
//    QTreeWidgetItem item3(list);
//
//    registerWidget.setHeaderItem(&item);
//
//    registerWidget.addTopLevelItem(&item2);
//    item2.addChild(&item3);
//    item.addChild(&item2);
    mw.show();
    return a.exec();
}
