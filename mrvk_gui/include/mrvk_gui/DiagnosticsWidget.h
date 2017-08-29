//
// Created by controller on 8/20/17.
//

#ifndef PROJECT_DIAGNOSTICS_H
#define PROJECT_DIAGNOSTICS_H

#include <ui_DiagnosticsWidget.h>
#include <QWidget>
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <mrvk_gui/DiagnosticModel.h>
#include <QTreeView>
#include <QStandardItemModel>

class DiagnosticsWidget {
public:
    DiagnosticsWidget();
    void setupUi(QWidget *widget);
    void diagnosticMsgCallback(const diagnostic_msgs::DiagnosticArray &msg);
    QModelIndex checkIfItemExist(const diagnostic_msgs::DiagnosticStatus &msg);


        protected:
    //Qt
    Ui::DiagnosticsWidget diagnosticsUi;
    QTreeView* treeView;
    QStandardItemModel model;
    QList<QString> levels;
    //DiagnosticModel treeModel;
    //QTreeView treeView;

    //ros
    ros::NodeHandle nh;
    ros::Subscriber diagnosticsSub;
};


#endif //PROJECT_DIAGNOSTICS_H
