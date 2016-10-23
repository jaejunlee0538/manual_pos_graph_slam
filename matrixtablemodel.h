#ifndef MATRIXTABLEMODEL_H
#define MATRIXTABLEMODEL_H
#include "Global.h"
#include <QAbstractTableModel>
#include <QVector>

class MatrixTableModel : public QAbstractTableModel
{
protected:
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

        //TODO : deprecated.
        const double& getDoubleData() const{
            return data;
        }

    private:
        double data;
        bool valid;
    };
    typedef QVector<Element> RowType;
public:


    explicit MatrixTableModel(QObject * parent = 0);
    bool getMatrix(GeneralMatrixType& output);
    void setMatrix(const GeneralMatrixType &m);
    bool setMatrices(const QVector<GeneralMatrixType> &matrices);
    bool isEmpty()const;
    ////////////////////////////////////////
    //Check if matrix isn't empty and every element has valid data(double)
    bool isValid()const;
    int rowCount(const QModelIndex &parent=QModelIndex()) const;
    int columnCount(const QModelIndex &parent=QModelIndex()) const;
    QVariant data(const QModelIndex &index, int role) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    Qt::ItemFlags flags(const QModelIndex &index) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role);

    //////////////////////////////////////////////////
protected:
    static void copyMatrix(const GeneralMatrixType &input, QVector<RowType>& output);
protected:
    bool isIndexInRange(const QModelIndex& index) const;
    QVector<RowType> matrix_data;
};

#endif // MATRIXTABLEMODEL_H
