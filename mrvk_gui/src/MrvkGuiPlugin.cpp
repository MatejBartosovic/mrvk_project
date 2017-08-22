//
// Created by matejko on 15.8.2017.
//

#include "mrvk_gui/MrvkGuiPlugin.h"
#include <pluginlib/class_list_macros.h>

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
        diagnosticsWidget.setupUi(mainUi.tabWidget->widget(0));

        //add to cintext
        context.addWidget(mainWidget);

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

}; // namespace
PLUGINLIB_EXPORT_CLASS(mrvk_gui::MrvkGui, rqt_gui_cpp::Plugin)