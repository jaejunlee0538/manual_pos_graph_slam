#ifndef MATRIXMANIPULATOR_H
#define MATRIXMANIPULATOR_H

#include <QWidget>
#include <QAbstractTableModel>
#include <Eigen/Core>
#include <QList>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
namespace Ui {
class MatrixManipulator;
}

class MatrixManipulator : public QWidget
{
    Q_OBJECT

protected:
    class MatrixTableModel : public QAbstractTableModel
    {
    public:
        class Element{
        public:
            Element():valid(false),data(0.0){

            }

            bool isValid() const{
                return valid;
            }

            void clear(){
                this->valid = false;
            }

            void setData(const double& data){
                this->data = data;
                this->valid = true;
            }

            QVariant getData() const{
                if(valid)
                    return QVariant(data);
                return QVariant(QString(""));
            }

            const double& getDoubleData() const{
                return data;
            }

        private:
            double data;
            bool valid;
        };

        typedef QVector<Element> RowType;
        typedef QVector<RowType> MatrixType;
        static void copyMatrix(const g2o::EdgeSE3::InformationType &input, MatrixType& output);
        bool toEigenMatrix(g2o::EdgeSE3::InformationType& output);
    public:
        explicit MatrixTableModel(QObject * parent = 0);
        void setMatrix(const g2o::EdgeSE3::InformationType &m);
        void setMatrix(const MatrixType& m);

        //Check if matrix isn't empty and every element has valid data(double)
        bool isValid()const;
        int rowCount(const QModelIndex &parent) const;
        int columnCount(const QModelIndex &parent) const;
        QVariant data(const QModelIndex &index, int role) const;
        QVariant headerData(int section, Qt::Orientation orientation, int role) const;
        Qt::ItemFlags flags(const QModelIndex &index) const;
        bool setData(const QModelIndex &index, const QVariant &value, int role);
    protected:
        bool isIndexInRange(const QModelIndex& index) const;
        MatrixType matrix_data;
    };

public:
    explicit MatrixManipulator(QWidget *parent = 0);
    ~MatrixManipulator();

public Q_SLOTS:
    void setMatrix(const g2o::EdgeSE3::InformationType& matrix);
    void setMatrices(const QList<g2o::EdgeSE3::InformationType> &matrices);

Q_SIGNALS:
    //emitted when 'Apply' button is pressed.
    void matrixModified(const g2o::EdgeSE3::InformationType& m);

private Q_SLOTS:
    void on_pushButton_Apply_clicked();
    void on_pushButton_Identity_clicked();
    void on_pushButton_Undo_clicked();

private:
    Ui::MatrixManipulator *ui;

    MatrixTableModel * model;
    g2o::EdgeSE3::InformationType original_matrix;
};

#endif // MATRIXMANIPULATOR_H
