#include "graphtablemodel.h"
#include <sstream>
#include <g2o/types/slam2d/types_slam2d.h>
QString vectorTostring(const QVector<double>& vec){
    if(vec.empty()){
        return QString("");
    }
    QString str;
    str.append(QString("%1").arg(vec[0]));
    for(size_t i=1;i<vec.size();i++){
        str.append(QString(", %1").arg(vec[i]));
    }
    return str;
}

EdgeTableModel::EdgeTableModel()
{

}

void EdgeTableModel::pushBack(const g2o::OptimizableGraph::Edge *edge)
{
    edges_se3.push_back(edge);
}

void EdgeTableModel::clear()
{
    edges_se3.clear();
}

const g2o::OptimizableGraph::Edge *EdgeTableModel::at(int irow) const
{
    return edges_se3[irow];
}

int EdgeTableModel::rowCount(const QModelIndex &parent) const
{
    return edges_se3.size();
}

int EdgeTableModel::columnCount(const QModelIndex &parent) const
{
    //id, vi, vj, measurement, info
    return 5;
}

QVariant EdgeTableModel::data(const QModelIndex &index, int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        const g2o::OptimizableGraph::Edge* edge = edges_se3[index.row()];
        switch(index.column()){
        case 0:                 return QVariant(edge->id());
        case 1:                 return QVariant(edge->vertex(0)->id());
        case 2:                 return QVariant(edge->vertex(1)->id());
        case 3:
        {
            QVector<double> data(edge->measurementDimension());
            edge->getMeasurementData(&data[0]);
            return QVariant(vectorTostring(data));
        }
        case 4:
        {
            int dim = edge->dimension();
            QVector<double> info(dim*dim);
            edge->informationData();
            std::copy(edge->informationData(), edge->informationData()+dim*dim, info.begin());
            return QVariant(vectorTostring(info));
        }
        }
    }
    return QVariant();
}

QVariant EdgeTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole ){
        if(orientation==Qt::Horizontal){
            switch(section){
            case 0:     return QVariant("ID");
            case 1:     return QVariant("Vi");
            case 2:     return QVariant("Vj");
            case 3:     return QVariant("Measurement");
            case 4:     return QVariant("Information");
            }
        }else if(orientation == Qt::Vertical){
            return QVariant(section);
        }
    }
    return QVariant();
}

Qt::ItemFlags EdgeTableModel::flags(const QModelIndex &index) const
{
    if(!index.isValid()){
        return Qt::ItemIsEnabled;
    }
    switch(index.column()){
    case 4:
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
    }
    return QAbstractItemModel::flags(index);
}

bool EdgeTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if(index.isValid() && role==Qt::EditRole){

        return true;
    }
    return false;
}


void VertexTableModel::pushBack(const g2o::VertexSE3 *vertex)
{
    vertices_se3.push_back(vertex);
}

const g2o::VertexSE3 *VertexTableModel::at(int irow) const
{
    return vertices_se3[irow];
}
