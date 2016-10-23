#ifndef EDGEMANIPULATOR_H
#define EDGEMANIPULATOR_H
#include <QWidget>
#include "graphtablemodel.h"
#include <memory>
#include <QItemSelectionModel>
#include "Global.h"
#include "edgemodification.h"

namespace Ui {
class EdgeManipulator;
}

typedef QList<EdgeModifyAction::Ptr> EdgeModifications;

class EdgeManipulator : public QWidget
{
    Q_OBJECT
public:
    explicit EdgeManipulator(QWidget *parent = 0);
    ~EdgeManipulator();

Q_SIGNALS:
    void edgesShouldBeRemoved(const QList<const g2o::OptimizableGraph::Edge*>& edges);
    void edgesShouldBeModified(const EdgeModifications& modifications);

public Q_SLOTS:
    void setEdges(EdgeTableModel* new_table);
    void selectRows(const QList<int>& rows);

private Q_SLOTS:
    //Delete selected edges
    void on_pushButton_Delete_clicked();
    //Change information matrix of selected edges.
    void slot_changeInformationMatrix(const GeneralMatrixType& new_mat);
    void updateSelections(QItemSelection selected,QItemSelection deselected);
    void on_pushButton_RobustKernel_Apply_clicked();

protected:
    EdgeTableModel* edge_table;

private:
    Ui::EdgeManipulator *ui;
};




#endif // EDGEMANIPULATOR_H
