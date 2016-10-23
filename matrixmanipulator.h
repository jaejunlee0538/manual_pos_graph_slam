#ifndef MATRIXMANIPULATOR_H
#define MATRIXMANIPULATOR_H

#include <QWidget>
#include <QAbstractTableModel>
#include <Eigen/Core>
#include <QList>
#include "matrixtablemodel.h"
namespace Ui {
class MatrixManipulator;
}

class MatrixManipulator : public QWidget
{
    Q_OBJECT

public:
    explicit MatrixManipulator(QWidget *parent = 0);
    ~MatrixManipulator();

public Q_SLOTS:
    void setMatrix(const GeneralMatrixType& matrix);
    bool setMatrices(const QVector<GeneralMatrixType> &matrices);

Q_SIGNALS:
    //emitted when 'Apply' button is pressed.
    void matrixModified(const GeneralMatrixType& m);

private Q_SLOTS:
    void on_pushButton_Apply_clicked();
    void on_pushButton_Identity_clicked();
    void on_pushButton_Undo_clicked();

private:
    Ui::MatrixManipulator *ui;
    MatrixTableModel * model;
    GeneralMatrixType original_matrix;
};

#endif // MATRIXMANIPULATOR_H
