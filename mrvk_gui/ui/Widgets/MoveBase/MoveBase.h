#ifndef MOVEBASE_H
#define MOVEBASE_H

#include <QWidget>

namespace Ui {
class MoveBase;
}

namespace mrvk_gui {
    class MoveBase : public QWidget {
    Q_OBJECT

    public:
        explicit MoveBase(QWidget* parent = 0);

        ~MoveBase();

    private:
        Ui::MoveBase* ui;
    };
}
#endif // MOVEBASE_H
