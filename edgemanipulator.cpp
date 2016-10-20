#include "edgemanipulator.h"
#include "ui_edgemanipulator.h"
#include "Logger.h"

#define EDGE_MANIPULATOR_CONSOLE_DEBUG

EdgeManipulator::EdgeManipulator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EdgeManipulator),edge_table(nullptr)
{
    ui->setupUi(this);
    QObject::connect(ui->widget_MatrixManipulator, SIGNAL(matrixModified(g2o::EdgeSE3::InformationType)),
                     this, SLOT(slot_changeInformationMatrix(g2o::EdgeSE3::InformationType)));
}

EdgeManipulator::~EdgeManipulator()
{
    delete ui;
}

void EdgeManipulator::setEdges(EdgeTableModel *new_table)
{
    DEBUG_FUNC_STAMP;
    if(new_table == nullptr){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"new_table is nullptr["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    if(edge_table){
        delete edge_table;
    }
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
    std::cerr<<new_table->rowCount()<<" edges are received["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
    edge_table = new_table;
    ui->tableView_Edges->setModel(edge_table);
    //This is bad. When the model is switched, the signal connection cannot be avaliable....
    //Think about just leaving the model as it is and just copying data into the model.
        ui->tableView_Edges->setSelectionMode(QTableView::MultiSelection);
    QObject::connect(ui->tableView_Edges->selectionModel(), SIGNAL(selectionChanged(QItemSelection,QItemSelection)),
                     this, SLOT(updateSelections(QItemSelection,QItemSelection)));
}

void EdgeManipulator::selectRows(const QList<int> &rows)
{
    ui->tableView_Edges->clearSelection();
    DEBUG_FUNC_STAMP;

    for(const auto& idx:rows){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<idx<<" row selected"<<std::endl;
#endif
        ui->tableView_Edges->selectRow(idx);
    }
}

void EdgeManipulator::on_pushButton_Delete_clicked()
{
    DEBUG_FUNC_STAMP;
    QModelIndexList selected_row = ui->tableView_Edges->selectionModel()->selectedRows();
    if(selected_row.empty()){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"Selected row is empty["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    QList<const g2o::EdgeSE3*> edges_to_remove;
    for(size_t i=0;i<selected_row.size();i++){
        edges_to_remove.push_back(edge_table->at(selected_row[i].row()));
    }
    Q_EMIT edgesShouldBeRemoved(edges_to_remove);
}

void EdgeManipulator::slot_changeInformationMatrix(const g2o::EdgeSE3::InformationType &new_mat)
{
    DEBUG_FUNC_STAMP;
    QModelIndexList selected_row = ui->tableView_Edges->selectionModel()->selectedRows();
    if(selected_row.empty()){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"Selected row is empty["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    QList<EdgeModifyAction::Ptr> modifications;
    for(size_t i=0;i<selected_row.size();i++){
        modifications.push_back(
                    EdgeModifyAction::Ptr(new EdgeModifyActionChangeInformation(
                                              edge_table->at(selected_row[i].row()), new_mat)));
    }
    Q_EMIT edgesShouldBeModified(modifications);
}

void EdgeManipulator::updateSelections(QItemSelection selected, QItemSelection deselected)
{
    DEBUG_FUNC_STAMP;
    QModelIndexList selected_row = ui->tableView_Edges->selectionModel()->selectedRows();
    if(selected_row.empty()){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"Selected row is empty["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    QList<g2o::EdgeSE3::InformationType> matrices;
    g2o::EdgeSE3::InformationType m;

    for(size_t i=0;i<selected_row.size();i++){
        matrices.push_back(edge_table->at(selected_row[i].row())->information());
    }
    ui->widget_MatrixManipulator->setMatrices(matrices);
}

