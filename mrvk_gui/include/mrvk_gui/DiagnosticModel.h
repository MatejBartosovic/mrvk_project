//
// Created by controller on 8/20/17.
//

#ifndef PROJECT_DIAGNOSTICMODEL_H
#define PROJECT_DIAGNOSTICMODEL_H

#include <QAbstractItemModel>
#include <QModelIndex>
#include <QVariant>
#include <mrvk_gui/DiagnosticItem.h>
#include <diagnostic_msgs/DiagnosticArray.h>

class DiagnosticModel : public QAbstractItemModel
{
Q_OBJECT

public:
    DiagnosticModel(QObject *parent = 0);
    ~DiagnosticModel();

    //The constructor and destructor are specific to this model.

    QVariant data(const QModelIndex &index, int role) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;

    QModelIndex index(int row, int column,
                      const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &index) const override;

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    //Read-only tree models only need to provide the above functions. The following public functions provide support for editing and resizing:

            Qt::ItemFlags flags(const QModelIndex &index) const override;
    bool setData(const QModelIndex &index, const QVariant &value,
                 int role = Qt::EditRole) override;
    bool setHeaderData(int section, Qt::Orientation orientation,
                       const QVariant &value, int role = Qt::EditRole) override;

    bool insertColumns(int position, int columns,
                       const QModelIndex &parent = QModelIndex()) override;
    bool removeColumns(int position, int columns,
                       const QModelIndex &parent = QModelIndex()) override;
    bool insertRows(int position, int rows,
                    const QModelIndex &parent = QModelIndex()) override;
    bool removeRows(int position, int rows,
                    const QModelIndex &parent = QModelIndex()) override;

    DiagnosticItem* createNewItemsFromMsg(diagnostic_msgs::DiagnosticStatus &msg);
    void registerNewMsg(diagnostic_msgs::DiagnosticStatus msg);
    void fillWithMsg(diagnostic_msgs::DiagnosticStatus &msg, DiagnosticItem* item);
private:
    //void setupModelData(const QStringList &lines, DiagnosticModel *parent);
    DiagnosticItem *getItem(const QModelIndex &index) const;

    DiagnosticItem *rootItem;
};

#endif //PROJECT_DIAGNOSTICMODEL_H
