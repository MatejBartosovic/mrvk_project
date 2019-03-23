#ifndef MOVEBASE_H
#define MOVEBASE_H

#include <QWidget>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace Ui {
class MoveBase;
}

namespace mrvk_gui {
    class MoveBase : public QWidget {
    Q_OBJECT

    public:
        explicit MoveBase(QWidget* parent = 0);

        ~MoveBase();

        void updateData();

    private:
        Ui::MoveBase* ui;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> actionClient;
    public slots:
        void goSlot();
        void cancelSlot();
        void editMapOffsetSlot();
        void readQrCodeSlot();
    };
}
#endif // MOVEBASE_H
