//
// Created by controller on 8/20/17.
//

#include <diagnostic_msgs/DiagnosticArray.h>
#include "mrvk_gui/DiagnosticsWidget.h"

DiagnosticsWidget::DiagnosticsWidget() : nh("~") {
    diagnosticsSub = nh.subscribe("/diagnostics",1,&DiagnosticsWidget::diagnosticMsgCallback,this);
}

void DiagnosticsWidget::setupUi(QWidget *widget) {
    // fix na tie modre vypisi
    qRegisterMetaType<QVector<int> >("QVector<int>");
    diagnosticsUi.setupUi(widget);
    treeView = diagnosticsUi.diagnosticsTreeView;
    treeView->setModel(&model);
    treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);

    levels << "Ok" << "Warning" << "Error" << "Stale";
    QList<QStandardItem*> row;
    for(int i = 0; i < levels.size(); i++){
        row << new QStandardItem(levels[i]) << new QStandardItem("(0)");
        model.appendRow(row);
        row.clear();
    }
}

void DiagnosticsWidget::diagnosticMsgCallback(const diagnostic_msgs::DiagnosticArray &msg){
    ROS_ERROR("new msg");
    int dataRows = 0;
    for(int i = 0; i<msg.status.size(); i++) {
        QList<QStandardItem*> row;
        checkIfItemExist(msg.status[i]);
        QModelIndex rootIndex = checkIfItemExist(msg.status[i]);
        if (rootIndex.isValid()){
            QModelIndex sibling(rootIndex.sibling(rootIndex.row(),1));
            if(sibling.isValid()){
                //toto sposobuje tie modre vypisi v konstruktore je fix ale neviem preco vznikali
                model.setData(sibling,msg.status[i].message.c_str());
            }
            for(int j = 0 ; j < msg.status[i].values.size();j++){
                QModelIndex childIndex = rootIndex.child(j,0);
                if(!childIndex.isValid()){
                    ROS_WARN("pridavam riadky mozno to spadne");
                    model.insertRow(j,rootIndex);
                }
                childIndex = rootIndex.child(j,0);
                if(childIndex.isValid()){
                    model.setData(childIndex,msg.status[i].values[j].key.c_str());
                    model.setData(childIndex.sibling(childIndex.row(),1),msg.status[i].values[j].value.c_str());
                }
                else{
                    ROS_ERROR("Hmmm toto by sa nikdy nemalo stat");
                }
                dataRows = j;
            }
            while (rootIndex.child(++dataRows,0).isValid()){
                ROS_WARN("odstrenujem riadky mozno to spadne");
                model.removeRow(dataRows,rootIndex);
            }
            continue;
        }

        row << new QStandardItem(msg.status[i].name.c_str()) << new QStandardItem(msg.status[i].message.c_str());
        model.item(msg.status[i].level,0)->appendRow(row);
        for(int j = 0; j < msg.status[i].values.size();j++){
            QList<QStandardItem*> values;
            values << new QStandardItem(msg.status[i].values[j].key.c_str()) << new QStandardItem(msg.status[i].values[j].value.c_str());
            row[0]->appendRow(values);
        }
    }
}

QModelIndex DiagnosticsWidget::checkIfItemExist(const diagnostic_msgs::DiagnosticStatus &msg){
    /*
     * check level in msg first
     * */
    for(int i = 0; i < model.item(msg.level,0)->rowCount(); i++) {
        std::string rrr = model.data(model.item(msg.level, 0)->child(i, 0)->index()).toString().toStdString();
        //ROS_ERROR("porovnavam %s %s",msg.name.c_str(),rrr.c_str());
        if (msg.name == rrr){
            return model.item(msg.level,0)->child(i,0)->index();
        }
    }
    /*
     * check odher levels and move if nesesary
     * */
    for (int i = 0; i < model.rowCount(); i++) {
        if(i == msg.level){
            continue;
        }
        for(int j = 0 ; j < model.item(i,0)->rowCount(); j++)
        if(msg.name == model.data(model.item(i,0)->child(j,0)->index()).toString().toStdString()){
            ROS_WARN("presuvam spravu mozno to spadne");
            model.item(i,0)->child(j,0)->clone();
            QList<QStandardItem*> row;
            row << model.item(i,0)->child(j,0)->clone() << model.item(i,0)->child(j,1)->clone();
            model.item(i,0)->removeRow(j);
            model.item(msg.level,0)->appendRow(row);
            return model.item(msg.level,0)->child(model.item(msg.level,0)->rowCount()-1,0)->index();
        }
    }
    return QModelIndex();
}