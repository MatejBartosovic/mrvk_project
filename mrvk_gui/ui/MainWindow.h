#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

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
        QTimer timer;
    public slots:
        void updateGui();

    };
}
#endif // MAINWINDOW_H
