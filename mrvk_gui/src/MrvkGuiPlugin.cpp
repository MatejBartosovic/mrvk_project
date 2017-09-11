//
// Created by matejko on 15.8.2017.
//

#include "mrvk_gui/MrvkGuiPlugin.h"
#include <pluginlib/class_list_macros.h>
#include <QMessageBox>

namespace mrvk_gui {

    MrvkGui::MrvkGui() : rqt_gui_cpp::Plugin(), mainWidget(0)
    {

    }

    void MrvkGui::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        QStringList argv = context.argv();

        //setup mainWidget
        mainWidget = new QWidget();
        mainUi.setupUi(mainWidget);

        //setup diagnosticsWidget
        //diagnosticsWidget.setupUi(mainUi.tabWidget->widget(0));
        controlWidget.setupUi(mainUi.tabWidget->widget(1));

        //add to cintext
        context.addWidget(mainWidget);

        connect(controlWidget.setGoal,SIGNAL(clicked()),this,SLOT(setGoal()));
        connect(controlWidget.cancelGoal,SIGNAL(clicked()),this,SLOT(cancelGoal()));

    }

    void MrvkGui::shutdownPlugin()
    {
        // TODO unregister all publishers here
    }

    void MrvkGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void MrvkGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }

    void MrvkGui::readNavigData(){

        if(controlWidget.decimal_check)

        longitude.stupne= controlWidget.long_stupne->toPlainText().toInt();
        longitude.min = controlWidget.long_minuty->toPlainText().toInt();
        longitude.sec = controlWidget.long_sekundy->toPlainText().toInt();

        latitude.stupne  = controlWidget.lat_stupne->toPlainText().toInt();
        latitude.min  = controlWidget.lat_minuty->toPlainText().toInt();
        latitude.sec  = controlWidget.lat_sekundy->toPlainText().toInt();

    }

    void MrvkGui::setGoal(){
        QMessageBox msgBox;
        msgBox.setText(QString("Do you want to start robot movement ?"));
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);

        switch (msgBox.exec()){
            case QMessageBox::Cancel:
                // Cancel was clicked
                break;
            case QMessageBox::Ok:
            default:
                // should never be reached
                break;
        }

    }

    void MrvkGui::cancelGoal(){
        QMessageBox msgBox;
        msgBox.setText(QString("Do you want to cancel robot movement ?"));
        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Cancel);
        switch (msgBox.exec()){
            case QMessageBox::Cancel:
                // Cancel was clicked
                break;
            case QMessageBox::Ok:
            default:
                // should never be reached
                break;
        }

    }


}; // namespace
PLUGINLIB_EXPORT_CLASS(mrvk_gui::MrvkGui, rqt_gui_cpp::Plugin)