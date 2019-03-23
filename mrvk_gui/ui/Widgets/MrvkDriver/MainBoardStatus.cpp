#include "MainBoardStatus.h"
#include <ui_MainBoardStatus.h>

using namespace std;
namespace mrvk_gui {
    MainBoardStatus::MainBoardStatus(QWidget *parent) :
            QWidget(parent),
            ui(new Ui::MainBoardStatus()) {

        ui->setupUi(this);
        ros::NodeHandle n("/");
        subscriber = new my_Subscriber("diagnostics", n);

        labelsMap.emplace("central_stop", ui->centralStopValue);
        labelsMap.emplace("hardware_central_stop", ui->hardwareCentralStopValue);
        labelsMap.emplace("temperature", ui->temperatureValue);
        labelsMap.emplace("power_off_sequence", ui->powerOffsequenceValue);
        labelsMap.emplace("full_battery", ui->fullBatteryValue);
        labelsMap.emplace("charge", ui->chargeValue);
        labelsMap.emplace("battery1_voltage", ui->battery1VoltageValue);
        labelsMap.emplace("battery2_voltage", ui->battery2VoltageValue);
        labelsMap.emplace("current", ui->currentValue);
        labelsMap.emplace("MCBsSB_5V", ui->MCBsSB5vValue);
        labelsMap.emplace("MCBs_12V", ui->MCBs12vValue);
        labelsMap.emplace("videotransmitter", ui->videoTransmitterValue);
        labelsMap.emplace("fan", ui->fanValue);
        labelsMap.emplace("laser", ui->laserValue);
        labelsMap.emplace("gps", ui->gpsValue);
        labelsMap.emplace("arm_5V", ui->arm5vValue);
        labelsMap.emplace("arm_12V", ui->arm12vValue);
        labelsMap.emplace("pc2", ui->pc2Value);
        labelsMap.emplace("camera", ui->cameraValue);
    }

    MainBoardStatus::~MainBoardStatus() {
        delete subscriber;
        delete ui;
    }

    void mrvk_gui::MainBoardStatus::updateData() {
        // TODO add colors for threshold values
        auto diagnostic_data = subscriber->getData();

        for (const auto & status: diagnostic_data.status) {
            if (status.name == MAIN_BOARD_STATUS_NAME) {
                for (const auto &data: status.values) {
                    auto labelElem = labelsMap.find(data.key);
                    labelElem->second->setText(QString::fromStdString(data.value));
                }
            }
        }
    }


}   // mrvk namespace