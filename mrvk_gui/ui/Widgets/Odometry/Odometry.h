#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <QWidget>

namespace Ui {
class Odometry;
}

namespace mrvk_gui {
    class Odometry : public QWidget {
    Q_OBJECT

    public:
        explicit Odometry(QWidget* parent = 0);

        ~Odometry();

    private:
        Ui::Odometry* ui;
    };
}
#endif // ODOMETRY_H
