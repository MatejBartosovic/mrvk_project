//
// Created by controller on 8/20/17.
//

#include "mrvk_gui/DiagnosticItem.h"

DiagnosticItem::DiagnosticItem(const QList<QVariant> data, DiagnosticItem *parent)
{
    parentItem = parent;
    itemData = data;
}

void DiagnosticItem::setParent(DiagnosticItem *parent){
    parentItem = parent;
}

DiagnosticItem::~DiagnosticItem()
{
    qDeleteAll(childItems);
}

DiagnosticItem *DiagnosticItem::parent()
{
    return parentItem;
}

DiagnosticItem *DiagnosticItem::child(int number)
{
    return childItems.value(number);
}

int DiagnosticItem::childCount() const
{
    return childItems.count();
}

int DiagnosticItem::childNumber() const
{
    if (parentItem)
        return parentItem->childItems.indexOf(const_cast<DiagnosticItem*>(this));

    return 0;
}

int DiagnosticItem::columnCount() const
{
    return itemData.count();
}

QVariant DiagnosticItem::data(int column) const
{
    return itemData.value(column);
}

bool DiagnosticItem::setData(int column, const QVariant &value)
{
    if (column < 0 || column >= itemData.size()){
        std::cout << "nie" <<std::endl;
        return false;
    }

    itemData[column] = value;
    return true;
}

bool DiagnosticItem::insertChildren(int position, int count, int columns)
{
    if (position < 0 || position > childItems.size())
        return false;

    for (int row = 0; row < count; ++row) {
        QList<QVariant> data;
        for(int i = 0 ;i < columns;i++) //init data to required size (hack)
            data << QVariant();
        data.reserve(columns);
        DiagnosticItem *item = new DiagnosticItem(data, this);
        childItems.insert(position, item);
    }

    return true;
}

bool DiagnosticItem::removeAndDeleteChildren(int position, int count)
{
    if (position < 0 || position + count > childItems.size())
        return false;

    for (int row = 0; row < count; ++row)
        delete childItems.takeAt(position);

    return true;
}

DiagnosticItem* DiagnosticItem::getAndRemoveChildren(int position)
{
    if (position < 0 || position > childItems.size())
        return nullptr;

    return childItems.takeAt(position);;
}

bool DiagnosticItem::insertColumns(int position, int columns)
{
    if (position < 0 || position > itemData.size())
        return false;

    for (int column = 0; column < columns; ++column)
        itemData.insert(position, QVariant());

            foreach (DiagnosticItem *child, childItems)
            child->insertColumns(position, columns);

    return true;
}

void DiagnosticItem::deleteAllChildren(){
    qDeleteAll(childItems);
}

void DiagnosticItem::insertChildren(DiagnosticItem* children){
    childItems.push_back(children);
    return;
}