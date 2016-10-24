#include "matrixmanipulator.h"
#include "ui_matrixmanipulator.h"
#include "Logger.h"

//#define MATRIX_MANIPULATOR_CONSOLE_DEBUG

MatrixManipulator::MatrixManipulator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MatrixManipulator), model(nullptr)
{
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Constructing MatrixManipulator");
#endif
    ui->setupUi(this);

#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Creating MatrixTableModel");
#endif
    model = new MatrixTableModel();
    ui->tableView_matrix->setModel(model);
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Constructing MatrixManipulator Done.");
#endif
}

MatrixManipulator::~MatrixManipulator()
{
    delete ui;
}

void MatrixManipulator::setMatrix(const GeneralMatrixType &matrix)
{
    original_matrix = matrix;
    model->setMatrix(original_matrix);
}

bool MatrixManipulator::setMatrices(const QVector<GeneralMatrixType> &matrices){
    return model->setMatrices(matrices);
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
        GeneralMatrixType m;
        model->getMatrix(m);
#ifdef MATRIX_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"New Matrix : \n"<<m<<std::endl;
#endif
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

