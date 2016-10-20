#ifndef GRAPHTABLEDIALOG_H
#define GRAPHTABLEDIALOG_H

#include <QDialog>
#include "graphtablemodel.h"
#include "edgemanipulator.h"
namespace Ui {
class GraphTableDialog;
}

class GraphTableDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GraphTableDialog(QWidget *parent = 0);
    ~GraphTableDialog();

Q_SIGNALS:
    void edgesShouldBeModified(const EdgeModifications& modifications);
    void edgesShouldBeRemoved(const QList<const g2o::EdgeSE3*>& edges_to_remove);
public Q_SLOTS:
    void setGraphTable(GraphTableData* graph_table);
    void selectVertices(const QList<int>& vertices);
    void selectEdges(const QList<int>& motion_edges, const QList<int>& loop_edges);

private Q_SLOTS:
    void slot_ModifyEdges(const EdgeModifications& modifications);
    void slot_RemoveEdges(const QList<const g2o::EdgeSE3*>& edges_to_remove);

private:
    Ui::GraphTableDialog *ui;
    GraphTableData * m_graph_table;
};

#endif // GRAPHTABLEDIALOG_H
