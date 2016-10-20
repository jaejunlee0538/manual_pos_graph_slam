#include "graphtabledialog.h"
#include "ui_graphtabledialog.h"
#include <QDebug>
#include "Logger.h"
#define GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
GraphTableDialog::GraphTableDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GraphTableDialog),
    m_graph_table(nullptr)
{
        DEBUG_FUNC_STAMP;
    ui->setupUi(this);

    //delete this dialog when ESC key or 'X' button is pressed.
    this->setAttribute(Qt::WA_DeleteOnClose);
    QObject::connect(
                ui->widget_LoopEdgesManipulator, SIGNAL(edgesShouldBeModified(EdgeModifications)),
                this, SLOT(slot_ModifyEdges(EdgeModifications)));
    QObject::connect(
                ui->widget_MotionEdgeManipulator, SIGNAL(edgesShouldBeModified(EdgeModifications)),
                this, SLOT(slot_ModifyEdges(EdgeModifications)));
    QObject::connect(
                ui->widget_LoopEdgesManipulator, SIGNAL(edgesShouldBeRemoved(QList<const g2o::EdgeSE3*>)),
                this, SLOT(slot_RemoveEdges(QList<const g2o::EdgeSE3*>)));
}

GraphTableDialog::~GraphTableDialog()
{
        DEBUG_FUNC_STAMP;
    delete ui;
}

void GraphTableDialog::setGraphTable(GraphTableData *graph_table)
{
        DEBUG_FUNC_STAMP;
    if(graph_table == nullptr){
        return;
    }
    m_graph_table = graph_table;
    if(m_graph_table->getLoopEdges()){
        ui->widget_LoopEdgesManipulator->setEdges(graph_table->getLoopEdges());
    }
    if(m_graph_table->getMotionEdges()){
        ui->widget_MotionEdgeManipulator->setEdges(graph_table->getMotionEdges());
    }
}

void GraphTableDialog::selectVertices(const QList<int>& vertices)
{
        DEBUG_FUNC_STAMP;
        DEBUG_MUST_NOT_RUN;
    //TODO
}

void GraphTableDialog::selectEdges(const QList<int> &motion_edges, const QList<int> &loop_edges)
{
    DEBUG_FUNC_STAMP;
    DEBUG_MUST_NOT_RUN;
    ui->widget_MotionEdgeManipulator->selectRows(motion_edges);
    ui->widget_LoopEdgesManipulator->selectRows(loop_edges);
}

void GraphTableDialog::slot_ModifyEdges(const EdgeModifications &modifications)
{
        DEBUG_FUNC_STAMP;
    Q_EMIT edgesShouldBeModified(modifications);
}

void GraphTableDialog::slot_RemoveEdges(const QList<const g2o::EdgeSE3 *> &edges_to_remove)
{
        DEBUG_FUNC_STAMP;
    Q_EMIT edgesShouldBeRemoved(edges_to_remove);
}


