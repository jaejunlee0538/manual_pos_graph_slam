#include "graphtablemodel.h"
#include <sstream>

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

void EdgeTableModel::pushBack(g2o::EdgeSE3 *edge)
{
    bin_edges.push_back(edge);
}

void EdgeTableModel::clear()
{
    bin_edges.clear();
}

g2o::EdgeSE3 *EdgeTableModel::at(int irow)
{
    return bin_edges[irow];
}

int EdgeTableModel::rowCount(const QModelIndex &parent) const
{
    return bin_edges.size();
}

int EdgeTableModel::columnCount(const QModelIndex &parent) const
{
    //id, vi, vj, measurement, info
    return 5;
}

QVariant EdgeTableModel::data(const QModelIndex &index, int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        const g2o::EdgeSE3* edge = bin_edges[index.row()];
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
            int dim = edge->measurementDimension();
            QVector<double> info(dim*dim);
            edge->informationData();
            std::copy(edge->informationData(), edge->informationData()+dim*dim, info.begin());
            return QVariant(vectorTostring(info));
        }
        default:
            return QVariant();
        }
        return QVariant();
    }
    return QVariant();
}

QVariant EdgeTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole && orientation==Qt::Horizontal){
        switch(section){
        case 0:     return QVariant("ID");
        case 1:     return QVariant("Vi");
        case 2:     return QVariant("Vj");
        case 3:     return QVariant("Measurement");
        case 4:     return QVariant("Information");
        default:
            return QVariant();
        }
        return QVariant();
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
