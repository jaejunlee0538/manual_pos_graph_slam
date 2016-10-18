#ifndef GRAPHTABLEMODEL_H
#define GRAPHTABLEMODEL_H
#include <QAbstractTableModel>
#include <g2o/types/slam3d/edge_se3.h>
//#include <g2o/core/base_vertex.h>
#include <memory>
class VertexTableModel : public QAbstractTableModel
{
public:
    VertexTableModel();
    int rowCount(const QModelIndex &parent) const;
    int columnCount(const QModelIndex &parent) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
};

class EdgeTableModel : public QAbstractTableModel
{
public:
    typedef std::shared_ptr<EdgeTableModel> Ptr;
public:
    EdgeTableModel();
    void pushBack(g2o::EdgeSE3* edge);
    void clear();
    g2o::EdgeSE3 *at(int irow);
    int rowCount(const QModelIndex &parent) const;
    int columnCount(const QModelIndex &parent) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
protected:
    QVector<g2o::EdgeSE3*> bin_edges;
};

#endif // GRAPHTABLEMODEL_H
