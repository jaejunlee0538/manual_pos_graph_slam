#include "matrixtablemodel.h"

MatrixTableModel::MatrixTableModel(QObject *parent)
    :QAbstractTableModel(parent){

}

void MatrixTableModel::copyMatrix(const GeneralMatrixType &input, QVector<RowType> &output)
{
    int rows = input.rows(), cols = input.cols();
    //check if output dimension is same with input dimension.
    if(output.size() != rows || output[0].size() !=cols){
        output.clear();
        RowType row(input.cols());
        for(int i=0;i<input.rows();i++){
            output.push_back(row);
        }
    }

    //copy data.
    for(int irow = 0; irow<rows;irow++){
        for(int icol=0;icol<cols;icol++){
            output[irow][icol].setData(input(irow, icol));
        }
    }
}

bool MatrixTableModel::getMatrix(GeneralMatrixType&output)
{
    if(!isValid()){
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"table data is invalid.["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return false;
    }
    int rows = rowCount(), cols = columnCount();
    output.resize(rows, cols);
    for(int irow = 0; irow<rows;irow++){
        for(int icol=0;icol<cols;icol++){
            output(irow,icol) = matrix_data[irow][icol].getDoubleData();
        }
    }
}

void MatrixTableModel::setMatrix(const GeneralMatrixType &m)
{
    copyMatrix(m, matrix_data);
}

bool doubleEqApprox(const double& v1, const double& v2, const double epsilon=1e-10){
    return fabs(v2-v1) < epsilon;
}


bool MatrixTableModel::setMatrices(const QVector<GeneralMatrixType> &matrices)
{
    int rows=matrices.first().rows(), cols=matrices.first().cols();
    for(auto iter = matrices.begin()+1;iter!=matrices.end();iter++){
        if(iter->rows() != rows || iter->cols() != cols){
            //matrices dimension are not equal
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
            std::cerr<<"matrices dimension are not equal["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
            this->matrix_data.clear();
            return false;
        }
    }

    const GeneralMatrixType& first = matrices.first();
    copyMatrix(first, matrix_data);
    for(auto iter = matrices.begin()+1;iter!=matrices.end();iter++){
        for(int irow=0;irow<rows;irow++){
            for(int icol=0;icol<cols;icol++ ){
                if(!doubleEqApprox(first(0,0), (*iter)(irow, icol))){
                    matrix_data[irow][icol].clear();
                }
            }
        }
    }
    return true;
}

bool MatrixTableModel::isEmpty() const
{
    if(rowCount()== 0 || columnCount()== 0)
        return true;
    return false;
}

bool MatrixTableModel::isValid() const
{
    if(isEmpty())
        return false;//empty data

    int rows = rowCount(), cols = columnCount();
    //element-wise validity check.
    for(int irow = 0; irow<rows;irow++){
        for(int icol=0;icol<cols;icol++){
            if(!matrix_data[irow][icol].isValid())
                return false;
        }
    }
    return true;
}

int MatrixTableModel::rowCount(const QModelIndex &parent) const
{
    if(isEmpty())
        return 1;
    return matrix_data.size();
}

int MatrixTableModel::columnCount(const QModelIndex &parent) const
{
    if(isEmpty())
        return 1;
    return matrix_data.first().size();
}

QVariant MatrixTableModel::data(const QModelIndex &index, int role) const
{
    if(isEmpty()){
        if(role==Qt::DisplayRole){
            return QVariant("Empty");
        }
    }else{
        if(role == Qt::DisplayRole || role == Qt::EditRole){
            return matrix_data[index.row()][index.column()].getData();
        }
    }
    return QVariant();
}

QVariant MatrixTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        switch(orientation){
        case Qt::Horizontal:
            if(section < 0 || section >=columnCount())
                return QVariant();
            return QVariant(section);
        case Qt::Vertical:
            if(section < 0 || section >=rowCount())
                return QVariant();
            return QVariant(section);
        }
    }
    return QVariant();
}

Qt::ItemFlags MatrixTableModel::flags(const QModelIndex &index) const
{
    if(!index.isValid()){
        return Qt::ItemIsEnabled;
    }
    if(isIndexInRange(index)){
        return QAbstractItemModel::flags(index)|Qt::ItemIsEditable;
    }
    return QAbstractItemModel::flags(index);
}

bool MatrixTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    if(index.isValid() && role == Qt::EditRole && isIndexInRange(index)){
        bool ok;
        double v = value.toDouble(&ok);
        if(ok){
            matrix_data[index.row()][index.column()].setData(v);
            return true;
        }
    }
    return false;
}

bool MatrixTableModel::isIndexInRange(const QModelIndex &index) const
{
    if(index.row()<0 || index.column()<0 || index.row() >= rowCount() || index.column() >= columnCount()){
        return false;
    }
    return true;
}
