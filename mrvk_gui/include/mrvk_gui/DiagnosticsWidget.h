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

class DiagnosticsWidget {
public:
    DiagnosticsWidget();
    void setupUi(QWidget *widget);
    void diagnosticMsgCallback(const diagnostic_msgs::DiagnosticArray &msg);

protected:
    //Qt
    Ui::DiagnosticsWidget diagnosticsUi;
    DiagnosticModel treeModel;

    //ros
    ros::NodeHandle nh;
    ros::Subscriber diagnosticsSub;
};


#endif //PROJECT_DIAGNOSTICS_H
