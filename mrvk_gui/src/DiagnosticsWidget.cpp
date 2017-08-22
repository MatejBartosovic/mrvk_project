//
// Created by controller on 8/20/17.
//

#include <diagnostic_msgs/DiagnosticArray.h>
#include "mrvk_gui/DiagnosticsWidget.h"

DiagnosticsWidget::DiagnosticsWidget() : nh("~") {
    diagnosticsSub = nh.subscribe("/diagnostics",1,&DiagnosticsWidget::diagnosticMsgCallback,this);
}

void DiagnosticsWidget::setupUi(QWidget *widget) {
    diagnosticsUi.setupUi(widget);
    diagnosticsUi.diagnosticsTreeView->setModel(&treeModel);
    //diagnosticsUi.diagnosticsTableView->setModel(treeModel);
}

void DiagnosticsWidget::diagnosticMsgCallback(const diagnostic_msgs::DiagnosticArray &msg){
    ROS_INFO("new data");
    /*for(size_t i = 0; i < msg.status.size(); i++){
        switch (msg.status[i].level) {
            case diagnostic_msgs::DiagnosticStatus::OK:
                break;
            case diagnostic_msgs::DiagnosticStatus::WARN:
                break;
            case diagnostic_msgs::DiagnosticStatus::ERROR:
                break;
            case diagnostic_msgs::DiagnosticStatus::STALE:
                break;
            default:
                ROS_WARN("Unknown diagnostic level received. (%d)",msg.status[i].level);
        }
    }*/
}