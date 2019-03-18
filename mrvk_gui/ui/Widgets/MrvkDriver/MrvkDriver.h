#ifndef MRVKDRIVER_H
#define MRVKDRIVER_H

#include <QWidget>

namespace Ui {
    class MrvkDriver;
}

namespace mrvk_gui {
    class MrvkDriver : public QWidget {
    Q_OBJECT

    public:
        explicit MrvkDriver(QWidget* parent = 0);

        ~MrvkDriver();

    private:
        Ui::MrvkDriver* ui;
    };
}
#endif // MRVKDRIVER_H
