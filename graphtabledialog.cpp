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
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Constructing GraphTableDialog");
#endif
    ui->setupUi(this);

    //delete this dialog when ESC key or 'X' button is pressed.
    this->setAttribute(Qt::WA_DeleteOnClose);

#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Setting up signals-slots");
#endif
    QObject::connect(
                ui->widget_LoopEdgesManipulator, SIGNAL(edgesShouldBeModified(EdgeModifications)),
                this, SLOT(slot_ModifyEdges(EdgeModifications)));
    QObject::connect(
                ui->widget_MotionEdgeManipulator, SIGNAL(edgesShouldBeModified(EdgeModifications)),
                this, SLOT(slot_ModifyEdges(EdgeModifications)));
    QObject::connect(
                ui->widget_LoopEdgesManipulator, SIGNAL(edgesShouldBeRemoved(QList<const g2o::EdgeSE3*>)),
                this, SLOT(slot_RemoveEdges(QList<const g2o::EdgeSE3*>)));
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Construction done.");
#endif
}

GraphTableDialog::~GraphTableDialog()
{
    delete ui;
}

void GraphTableDialog::setGraphTable(GraphTableData *graph_table)
{
    if(graph_table == nullptr){
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("graph_table is null");
#endif
        return;
    }
    m_graph_table = graph_table;
    if(m_graph_table->getLoopEdges()){
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("setting loop edges table");
#endif
        ui->widget_LoopEdgesManipulator->setEdges(graph_table->getLoopEdges());
    }
    if(m_graph_table->getMotionEdges()){
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("setting motion edges table");
#endif
        ui->widget_MotionEdgeManipulator->setEdges(graph_table->getMotionEdges());
    }
}

void GraphTableDialog::selectVertices(const QList<int>& vertices)
{
       ERROR_MESSAGE_WITH_FUNC_INFO("This method is not implemented!");
    //TODO
}

void GraphTableDialog::selectEdges(const QList<int> &motion_edges, const QList<int> &loop_edges)
{
    ui->widget_MotionEdgeManipulator->selectRows(motion_edges);
    ui->widget_LoopEdgesManipulator->selectRows(loop_edges);
}

void GraphTableDialog::slot_ModifyEdges(const EdgeModifications &modifications)
{
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_FUNC_STAMP;
#endif
    Q_EMIT edgesShouldBeModified(modifications);
}

void GraphTableDialog::slot_RemoveEdges(const QList<const g2o::EdgeSE3 *> &edges_to_remove)
{
#ifdef GRAPH_TABLE_DIALOG_CONSOLE_DEBUG
    DEBUG_FUNC_STAMP;
#endif
    Q_EMIT edgesShouldBeRemoved(edges_to_remove);
}


