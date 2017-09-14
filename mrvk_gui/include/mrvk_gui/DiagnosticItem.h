//
// Created by controller on 8/20/17.
//

#ifndef PROJECT_DIAGNOSTICITEM_H
#define PROJECT_DIAGNOSTICITEM_H

#include <QVariant>
#include <QList>
#include <std_msgs/Header.h>

class DiagnosticItem
{
public:
    explicit DiagnosticItem(const QList<QVariant> data, DiagnosticItem *parent = 0);
    ~DiagnosticItem();

    DiagnosticItem *child(int number);
    void setParent(DiagnosticItem *parent);
    int childCount() const;
    int columnCount() const;
    QVariant data(int column) const;
    bool insertChildren(int position, int count, int columns);
    void insertChildren(DiagnosticItem* children);
    bool insertColumns(int position, int columns);
    DiagnosticItem *parent();
    bool removeAndDeleteChildren(int position, int count);
    DiagnosticItem* getAndRemoveChildren(int position);
    bool removeColumns(int position, int columns);
    int childNumber() const;
    bool setData(int column, const QVariant &value);

    void deleteAllChildren();
private:
    QList<DiagnosticItem*> childItems;
    QList<QVariant> itemData;
    DiagnosticItem *parentItem;
};


#endif //PROJECT_DIAGNOSTICITEM_H
