#ifndef MOVEBASECONTROL_H
#define MOVEBASECONTROL_H

#include <QWidget>

namespace Ui {
class MoveBaseControl;
}

namespace mrvk_gui {
    class MoveBaseControl : public QWidget {
    Q_OBJECT

    public:
        explicit MoveBaseControl(QWidget* parent = 0);

        ~MoveBaseControl();

        void updateData();

        Ui::MoveBaseControl* ui;
    };
}
#endif // MOVEBASECONTROL_H
