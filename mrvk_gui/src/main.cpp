#include <MainWindow.h>
#include <QApplication>
#include <mrvk_gui/Subscriber.h>
#include <std_msgs/String.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    mrvk_gui::MainWindow w;
    w.show();

    return a.exec();
}
