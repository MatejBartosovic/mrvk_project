#include "TopicFrequency.h"
#include "ui_TopicFrequency.h"

namespace mrvk_gui {
    TopicFrequency::TopicFrequency(QWidget* parent) :
            QWidget(parent),
            ui(new Ui::TopicFrequency) {
        ui->setupUi(this);
    }

    TopicFrequency::~TopicFrequency() {
        delete ui;
    }
}