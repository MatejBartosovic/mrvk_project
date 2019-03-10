#ifndef TOPICFREQUENCY_H
#define TOPICFREQUENCY_H

#include <QWidget>

namespace Ui {
    class TopicFrequency;
}

namespace mrvk_gui {
    class TopicFrequency : public QWidget {
    Q_OBJECT

    public:
        explicit TopicFrequency(QWidget* parent = 0);

        ~TopicFrequency();

    private:
        Ui::TopicFrequency* ui;
    };
}
#endif // TOPICFREQUENCY_H
