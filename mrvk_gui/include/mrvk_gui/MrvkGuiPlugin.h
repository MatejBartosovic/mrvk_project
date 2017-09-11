//
// Created by matejko on 15.8.2017.
//

#ifndef PROJECT_MRVKGUIPLUGIN_H
#define PROJECT_MRVKGUIPLUGIN_H
#include <rqt_gui_cpp/plugin.h>
#include <ui_MainWidget.h>
#include <mrvk_gui/DiagnosticsWidget.h>
#include <ui_ControlWidget.h>
#include <osm_planner/osm_parser.h>


struct gps_data{
    int8_t stupne;
    int8_t min;
    int8_t sec;
    double value;
};

namespace mrvk_gui{

    class MrvkGui : public rqt_gui_cpp::Plugin {
    Q_OBJECT
    public:MrvkGui();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    private:

        gps_data latitude,longitude;
        Ui::MainWidget mainUi;
        QWidget* mainWidget;
        DiagnosticsWidget diagnosticsWidget;
        Ui::ControlWidget controlWidget;
        //DiagnosticModel treeModel;
        private slots:
        void setGoal();
        void cancelGoal();
        void readNavigData();
    };
};


#endif //PROJECT_MRVKGUIPLUGIN_H
