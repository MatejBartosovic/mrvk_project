#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

namespace mrvk_gui {
    class MainWindow : public QMainWindow {
    Q_OBJECT

    public:
        explicit MainWindow(QWidget* parent = 0);

        ~MainWindow();

    private:
        Ui::MainWindow* ui;
    };
}
#endif // MAINWINDOW_H
