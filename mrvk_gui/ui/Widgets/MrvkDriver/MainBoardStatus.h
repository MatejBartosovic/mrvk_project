#ifndef MAINBOARDSTATUS_H
#define MAINBOARDSTATUS_H

#include <QWidget>

namespace Ui {
class MainBoardStatus;
}

namespace mrvk_gui {
    class MainBoardStatus : public QWidget {
    Q_OBJECT

    public:
        explicit MainBoardStatus(QWidget* parent = 0);

        ~MainBoardStatus();

    private:
        Ui::MainBoardStatus* ui;
    };
}
#endif // MAINBOARDSTATUS_H
