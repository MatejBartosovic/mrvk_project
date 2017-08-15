//
// Created by matejko on 15.8.2017.
//

#include "mrvk_gui/MrvkGuiPlugin.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace mrvk_gui {

    MrvkGuiPlugin::MrvkGuiPlugin() : rqt_gui_cpp::Plugin(), widget_(0)
    {

    }

    void MrvkGuiPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        // access standalone command line arguments
        QStringList argv = context.argv();


        widget_ = new QWidget();
        ui.setupUi(widget_);
        context.addWidget(widget_);
    }

    void MrvkGuiPlugin::shutdownPlugin()
    {
        // TODO unregister all publishers here
    }

    void MrvkGuiPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        // TODO save intrinsic configuration, usually using:
        // instance_settings.setValue(k, v)
    }

    void MrvkGuiPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        // TODO restore intrinsic configuration, usually using:
        // v = instance_settings.value(k)
    }

}; // namespace
PLUGINLIB_EXPORT_CLASS(mrvk_gui::MrvkGuiPlugin, rqt_gui_cpp::Plugin)