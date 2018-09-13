//
// Created by matejko on 15.8.2017.
//

#ifndef PROJECT_MRVKGUIPLUGIN_H
#define PROJECT_MRVKGUIPLUGIN_H

// ui
#include <rqt_gui_cpp/plugin.h>
#include <ui_MainWindow.h>
#include <pluginlib/class_list_macros.h>
#include <mrvk_gui/RobotourControl.h>

namespace mrvk_gui{

    class MrvkGui : public rqt_gui_cpp::Plugin {
    Q_OBJECT
    public:
        MrvkGui();
        virtual void initPlugin(qt_gui_cpp::PluginContext &context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    private:
        Ui::MainWindow mainUi;

        RobotourControl robotourControl;
    };
}


#endif //PROJECT_MRVKGUIPLUGIN_H
