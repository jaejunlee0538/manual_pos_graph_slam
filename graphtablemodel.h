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
    void pushBack(const g2o::OptimizableGraph::Vertex* vertex);
    const g2o::OptimizableGraph::Vertex *at(int irow) const;
    int rowCount(const QModelIndex &parent= QModelIndex()) const;
    int columnCount(const QModelIndex &parent= QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
protected:
    QVector<const g2o::OptimizableGraph::Vertex*> vertices;
};
class EdgeTableModel : public QAbstractTableModel
{
public:
    typedef std::shared_ptr<EdgeTableModel> Ptr;
public:
    EdgeTableModel();
    void pushBack(const g2o::OptimizableGraph::Edge* edge);
    void clear();
    const g2o::OptimizableGraph::Edge *at(int irow) const;
    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount(const QModelIndex &parent= QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role);
protected:
    QVector<const g2o::OptimizableGraph::Edge*> edges_se3;
};

class GraphTableData{
public:
    GraphTableData():loop_edges(nullptr), motion_edges(nullptr),vertices(nullptr){

    }

    ~GraphTableData(){
        if(loop_edges)
            delete loop_edges;
        if(motion_edges)
            delete motion_edges;
        if(vertices)
            delete vertices;
    }

    void setVertices(VertexTableModel* vertices){
        this->vertices = vertices;
    }

    void setLoopEdges(EdgeTableModel* edges){
        loop_edges = edges;
    }

    void setMotionEdges(EdgeTableModel* edges){
        motion_edges = edges;
    }

    VertexTableModel* getVertices(){
        return vertices;
    }

    EdgeTableModel* getLoopEdges(){
        return loop_edges;
    }

    EdgeTableModel* getMotionEdges(){
        return motion_edges;
    }
protected:
    EdgeTableModel* loop_edges;
    EdgeTableModel* motion_edges;
    VertexTableModel * vertices;
};
#endif // GRAPHTABLEMODEL_H
