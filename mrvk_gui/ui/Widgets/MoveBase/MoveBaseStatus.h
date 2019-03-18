#ifndef MOVEBASESTATUS_H
#define MOVEBASESTATUS_H

#include <QWidget>

namespace Ui {
class MoveBaseStatus;
}

namespace mrvk_gui {
    class MoveBaseStatus : public QWidget {
    Q_OBJECT

    public:
        explicit MoveBaseStatus(QWidget* parent = 0);

        ~MoveBaseStatus();

    private:
        Ui::MoveBaseStatus* ui;
    };
}
#endif // MOVEBASESTATUS_H
