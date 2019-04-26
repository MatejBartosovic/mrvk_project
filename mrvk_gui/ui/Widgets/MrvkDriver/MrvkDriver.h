#ifndef MRVKDRIVER_H
#define MRVKDRIVER_H

#include <QWidget>
#include <mrvk_gui/MrvkControllButtons.h>

namespace Ui {
    class MrvkDriver;
}

namespace mrvk_gui {
    class MrvkDriver : public QWidget {
    Q_OBJECT

    public:
        explicit MrvkDriver(QWidget* parent = 0);

        ~MrvkDriver();
        void updateData();

    public slots:
        void setCentralStop();
        void resetCentralStop();
        void blockMovement();
        void unblockMovement();
        void autoComputeGPS();
        void drawRoads();

    private:
        Ui::MrvkDriver* ui;
        MrvkControllButtons mrvkButtons;
    };
}
#endif // MRVKDRIVER_H
