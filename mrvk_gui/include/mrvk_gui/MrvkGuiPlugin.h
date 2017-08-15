//
// Created by matejko on 15.8.2017.
//

#ifndef PROJECT_MRVKGUIPLUGIN_H
#define PROJECT_MRVKGUIPLUGIN_H
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <ui_MainWidget.h>

namespace mrvk_gui{
    class MrvkGui : public rqt_gui_cpp::Plugin
        {
            Q_OBJECT
            public:MrvkGui();
            virtual void initPlugin(qt_gui_cpp::PluginContext& context);
            virtual void shutdownPlugin();
            virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
            virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

            private:
            Ui::MainWidget ui;
            QWidget* widget_;

    };
};


#endif //PROJECT_MRVKGUIPLUGIN_H
