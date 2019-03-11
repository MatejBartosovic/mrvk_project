#include <MainWindow.h>
#include <QApplication>
#include <ros/ros.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mrvk_gui");
    QApplication a(argc, argv);
    mrvk_gui::MainWindow w;
    w.show();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    return a.exec();
}
