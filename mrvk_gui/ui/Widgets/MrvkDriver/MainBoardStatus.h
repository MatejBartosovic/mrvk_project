#ifndef MAINBOARDSTATUS_H
#define MAINBOARDSTATUS_H

#include <QWidget>
#include <QLabel>
#include <mrvk_gui/Subscriber.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <map>

#define MAIN_BOARD_STATUS_NAME "main board status"

namespace Ui {
class MainBoardStatus;
}
//class QLabel;

namespace mrvk_gui {
    class my_Subscriber: public Subscriber<diagnostic_msgs::DiagnosticArray> {
    public:
        my_Subscriber(std::string topic, ros::NodeHandle& nh):Subscriber(topic, nh) {}
    private:
        bool msgValidityCallback(TConstPtr msg) override {
            for (const auto &status: msg->status) {
                if (status.name == MAIN_BOARD_STATUS_NAME) {
                    return true;
                }
            }
            return false;
        }
    };


    class MainBoardStatus : public QWidget {
    Q_OBJECT

    public:
        explicit MainBoardStatus(QWidget* parent = 0);

        ~MainBoardStatus() override;
        void updateData();

    private:
        Ui::MainBoardStatus* ui;
        my_Subscriber *subscriber;
        std::map<std::string, QLabel*> labelsMap;

    };
}
#endif // MAINBOARDSTATUS_H
