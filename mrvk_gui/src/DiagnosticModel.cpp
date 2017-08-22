//
// Created by controller on 8/20/17.
//
#include "mrvk_gui/DiagnosticItem.h"
#include "mrvk_gui/DiagnosticModel.h"
#include <ros/ros.h>

DiagnosticModel::DiagnosticModel(QObject *parent)
        : QAbstractItemModel(parent)
{
    QList<QVariant> rootData;
    QList<QVariant> data;
    rootData << "Device";
    rootData << "Status";
    data << "Stale";
    data << "Errors";
    data << "Warnings";
    data << "Ok";
    rootItem = new DiagnosticItem(rootData);
    rootItem->insertChildren(rootItem->childCount(),data.size(),1);
    for (int i = 0; i < data.size(); i++)
        rootItem->child(i)->setData(0,data[i]);
}
DiagnosticModel::~DiagnosticModel()
{
    delete rootItem;
}

DiagnosticItem *DiagnosticModel::getItem(const QModelIndex &index) const
{
    if (index.isValid()) {
        DiagnosticItem *item = static_cast<DiagnosticItem*>(index.internalPointer());
        if (item)
            return item;
    }
    return rootItem;
}

int DiagnosticModel::rowCount(const QModelIndex &parent) const
{
    DiagnosticItem *parentItem = getItem(parent);

    return parentItem->childCount();
}

int DiagnosticModel::columnCount(const QModelIndex & /* parent */) const
{
    return rootItem->columnCount();
}

Qt::ItemFlags DiagnosticModel::flags(const QModelIndex &index) const
{
    if (!index.isValid())
        return 0;

    return Qt::ItemIsEditable | QAbstractItemModel::flags(index);
}

QModelIndex DiagnosticModel::index(int row, int column, const QModelIndex &parent) const
{
    if (parent.isValid() && parent.column() != 0)
        return QModelIndex();
    DiagnosticItem *parentItem = getItem(parent);

    DiagnosticItem *childItem = parentItem->child(row);
    if (childItem)
        return createIndex(row, column, childItem);
    else
        return QModelIndex();
}

QModelIndex DiagnosticModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    DiagnosticItem *childItem = getItem(index);
    DiagnosticItem *parentItem = childItem->parent();

    if (parentItem == rootItem)
        return QModelIndex();

    return createIndex(parentItem->childNumber(), 0, parentItem);
}

bool DiagnosticModel::removeRows(int position, int rows, const QModelIndex &parent)
{
    DiagnosticItem *parentItem = getItem(parent);
    bool success = true;

    beginRemoveRows(parent, position, position + rows - 1);
    success = parentItem->removeChildren(position, rows);
    endRemoveRows();

    return success;
}

QVariant DiagnosticModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant();

    if (role != Qt::DisplayRole && role != Qt::EditRole)
        return QVariant();

    DiagnosticItem *item = getItem(index);

    return item->data(index.column());
}

bool DiagnosticModel::setHeaderData(int section, Qt::Orientation orientation,
                              const QVariant &value, int role)
{
    if (role != Qt::EditRole || orientation != Qt::Horizontal)
        return false;

    bool result = rootItem->setData(section, value);

    if (result)
            emit headerDataChanged(orientation, section, section);

    return result;
}

bool DiagnosticModel::removeColumns(int position, int columns, const QModelIndex &parent)
{
    bool success;

    beginRemoveColumns(parent, position, position + columns - 1);
    success = rootItem->removeColumns(position, columns);
    endRemoveColumns();

    if (rootItem->columnCount() == 0)
        removeRows(0, rowCount());

    return success;
}

QVariant DiagnosticModel::headerData(int section, Qt::Orientation orientation,
                               int role) const
{
    if (orientation == Qt::Horizontal && role == Qt::DisplayRole)
        return rootItem->data(section);

    return QVariant();
}


bool DiagnosticModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if (role != Qt::EditRole)
        return false;

    DiagnosticItem *item = getItem(index);
    bool result = item->setData(index.column(), value);

    if (result)
            emit dataChanged(index, index);

    return result;
}

bool DiagnosticModel::insertColumns(int position, int columns, const QModelIndex &parent)
{
    bool success;

    beginInsertColumns(parent, position, position + columns - 1);
    success = rootItem->insertColumns(position, columns);
    endInsertColumns();

    return success;
}

bool DiagnosticModel::insertRows(int position, int rows, const QModelIndex &parent)
{
    DiagnosticItem *parentItem = getItem(parent);
    bool success;

    beginInsertRows(parent, position, position + rows - 1);
    success = parentItem->insertChildren(position, rows, rootItem->columnCount());
    endInsertRows();

    return success;
}

