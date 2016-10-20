#include "matrixmanipulator.h"
#include "ui_matrixmanipulator.h"
#include "Logger.h"
#define MATRIX_MANIPULATOR_CONSOLE_DEBUG
MatrixManipulator::MatrixManipulator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MatrixManipulator), model(nullptr)
{
    ui->setupUi(this);
    model = new MatrixTableModel();
    auto m = g2o::EdgeSE3::InformationType();
    m.setZero();
    model->setMatrix(m);
    ui->tableView_matrix->setModel(model);
}

MatrixManipulator::~MatrixManipulator()
{
    delete ui;
}

void MatrixManipulator::setMatrix(const g2o::EdgeSE3::InformationType &matrix)
{
    DEBUG_FUNC_STAMP;
    //    MatrixTableModel* model = new MatrixTableModel();
    original_matrix = matrix;
    model->setMatrix(original_matrix);
}

bool doubleEqApprox(const double& v1, const double& v2, const double epsilon=1e-10){
    return fabs(v2-v1) < epsilon;
}

void MatrixManipulator::setMatrices(const QList<g2o::EdgeSE3::InformationType> &matrices)
{
    DEBUG_FUNC_STAMP;
    int rows=matrices.first().rows(), cols=matrices.first().cols();
    for(QList<g2o::EdgeSE3::InformationType>::const_iterator iter = matrices.begin()+1;iter!=matrices.end();iter++){
        if(iter->rows() != rows || iter->cols() != cols)//matrices dimension are not equal
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
            std::cerr<<"matrices dimension are not equal["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }

    MatrixTableModel::MatrixType matrix;
    MatrixTableModel::copyMatrix(matrices.first(), matrix);
    for(QList<g2o::EdgeSE3::InformationType>::const_iterator iter = matrices.begin()+1;iter!=matrices.end();iter++){
        for(int irow=0;irow<rows;irow++){
            for(int icol=0;icol<cols;icol++ ){
                if(!doubleEqApprox(matrix[irow][icol].getDoubleData(), (*iter)(irow, icol))){
                    matrix[irow][icol].clear();
                }
            }
        }
    }
    model->setMatrix(matrix);
}

void MatrixManipulator::on_pushButton_Apply_clicked()
{
    if(model == nullptr){
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"model is nullptr.["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    if(model->isValid()){
        g2o::EdgeSE3::InformationType m;
        model->toEigenMatrix(m);
        Q_EMIT matrixModified(m);
    }
}

void MatrixManipulator::on_pushButton_Identity_clicked()
{
    if(original_matrix.rows() == original_matrix.cols()){
        auto m = original_matrix;
        m.setZero();
        for(int i=0;i<m.rows();i++){
            m(i,i) = 1.0;
        }
        this->setMatrix(m);
    }
}

void MatrixManipulator::on_pushButton_Undo_clicked()
{
    this->setMatrix(original_matrix);
}
///////////////////////////TableModel Definitions/////////////////////////////////////////
void MatrixManipulator::MatrixTableModel::copyMatrix(const g2o::EdgeSE3::InformationType &input, MatrixManipulator::MatrixTableModel::MatrixType &output)
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

bool MatrixManipulator::MatrixTableModel::toEigenMatrix(g2o::EdgeSE3::InformationType &output)
{
    if(!isValid()){
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"table data is invalid.["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return false;
    }
    int rows = rowCount(QModelIndex()), cols = columnCount(QModelIndex());
    output.resize(rows, cols);
    for(int irow = 0; irow<rows;irow++){
        for(int icol=0;icol<cols;icol++){
            output(irow,icol) = matrix_data[irow][icol].getDoubleData();
        }
    }
}

MatrixManipulator::MatrixTableModel::MatrixTableModel(QObject *parent)
    :QAbstractTableModel(parent){

}

void MatrixManipulator::MatrixTableModel::setMatrix(const g2o::EdgeSE3::InformationType &m)
{
    copyMatrix(m, matrix_data);
}

void MatrixManipulator::MatrixTableModel::setMatrix(const MatrixManipulator::MatrixTableModel::MatrixType &m)
{
    matrix_data = m;
}

bool MatrixManipulator::MatrixTableModel::isValid() const
{
    int rows = rowCount(QModelIndex()), cols = columnCount(QModelIndex());
    if(rows == 0 || cols == 0)
        return false;//empty data
    //copy data.
    for(int irow = 0; irow<rows;irow++){
        for(int icol=0;icol<cols;icol++){
            if(!matrix_data[irow][icol].isValid())
                return false;
        }
    }
    return true;
}

int MatrixManipulator::MatrixTableModel::rowCount(const QModelIndex &parent) const
{
    return matrix_data.size();
}

int MatrixManipulator::MatrixTableModel::columnCount(const QModelIndex &parent) const
{
    if(matrix_data.empty()){
        return 0;
    }
    return matrix_data.first().size();
}

QVariant MatrixManipulator::MatrixTableModel::data(const QModelIndex &index, int role) const
{
    if(role == Qt::DisplayRole || role == Qt::EditRole){
        if(!isIndexInRange(index)){
            //Error : index out of range.
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
            std::cerr<<"Index out of range.["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
            return QVariant();
        }
        return matrix_data[index.row()][index.column()].getData();
    }
    return QVariant();
}

QVariant MatrixManipulator::MatrixTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole){
        switch(orientation){
        case Qt::Horizontal:
            if(section < 0 || section >=columnCount(QModelIndex()))
                return QVariant();
            return QVariant(section);
        case Qt::Vertical:
            if(section < 0 || section >=rowCount(QModelIndex()))
                return QVariant();
            return QVariant(section);
        }
    }
    return QVariant();
}

Qt::ItemFlags MatrixManipulator::MatrixTableModel::flags(const QModelIndex &index) const
{
    if(!index.isValid()){
        return Qt::ItemIsEnabled;
    }
    if(isIndexInRange(index)){
        return QAbstractItemModel::flags(index)|Qt::ItemIsEditable;
    }
    return QAbstractItemModel::flags(index);
}

bool MatrixManipulator::MatrixTableModel::setData(const QModelIndex &index, const QVariant &value, int role)
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

bool MatrixManipulator::MatrixTableModel::isIndexInRange(const QModelIndex &index) const
{
    if(index.row()<0 || index.column()<0 || index.row() >= rowCount(QModelIndex()) || index.column() >= columnCount(QModelIndex())){
        return false;
    }
    return true;
}
