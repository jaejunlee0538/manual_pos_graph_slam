#include "edgemanipulator.h"
#include "ui_edgemanipulator.h"
#include "Logger.h"
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include "graphhelper.h"
#define EDGE_MANIPULATOR_CONSOLE_DEBUG
namespace g2o{
static g2o::RegisterRobustKernelProxy<RobustKernelHuber> robust_kernel_register_Huber("Huber");
//G2O_REGISTER_ROBUST_KERNEL(Huber, RobustKernelHuber)
//G2O_REGISTER_ROBUST_KERNEL(PseudoHuber, RobustKernelPseudoHuber)
//G2O_REGISTER_ROBUST_KERNEL(Cauchy, RobustKernelCauchy)
//G2O_REGISTER_ROBUST_KERNEL(GemanMcClure, RobustKernelGemanMcClure)
//G2O_REGISTER_ROBUST_KERNEL(Welsch, RobustKernelWelsch)
//G2O_REGISTER_ROBUST_KERNEL(Fair, RobustKernelFair)
//G2O_REGISTER_ROBUST_KERNEL(Tukey, RobustKernelTukey)
//G2O_REGISTER_ROBUST_KERNEL(Saturated, RobustKernelSaturated)
//G2O_REGISTER_ROBUST_KERNEL(DCS, RobustKernelDCS)
}
EdgeManipulator::EdgeManipulator(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EdgeManipulator),edge_table(nullptr)
{
    ui->setupUi(this);
    std::vector<std::string> kernels;
    g2o::RobustKernelFactory::instance()->fillKnownKernels(kernels);
    ui->comboBox_RobustKernel->addItem(QString("None"));
    for(const auto& kernel:kernels){
        ui->comboBox_RobustKernel->addItem(QString(kernel.c_str()));
    }
    ui->comboBox_RobustKernel->setCurrentIndex(0);
    QObject::connect(ui->widget_MatrixManipulator, SIGNAL(matrixModified(GeneralMatrixType)),
                     this, SLOT(slot_changeInformationMatrix(GeneralMatrixType)));
}

EdgeManipulator::~EdgeManipulator()
{
    delete ui;
}

void EdgeManipulator::setEdges(EdgeTableModel *new_table)
{
//    DEBUG_FUNC_STAMP;
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
//    DEBUG_FUNC_STAMP;

    for(const auto& idx:rows){
        ui->tableView_Edges->selectRow(idx);
    }
}

void EdgeManipulator::on_pushButton_Delete_clicked()
{
//    DEBUG_FUNC_STAMP;
    QModelIndexList selected_row = ui->tableView_Edges->selectionModel()->selectedRows();
    if(selected_row.empty()){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"Selected row is empty["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    QList<const g2o::OptimizableGraph::Edge*> edges_to_remove;
    for(size_t i=0;i<selected_row.size();i++){
        edges_to_remove.push_back(edge_table->at(selected_row[i].row()));
    }
    Q_EMIT edgesShouldBeRemoved(edges_to_remove);
}

void EdgeManipulator::slot_changeInformationMatrix(const GeneralMatrixType &new_mat)
{
//    DEBUG_FUNC_STAMP;
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
//    DEBUG_FUNC_STAMP;
    QModelIndexList selected_row = ui->tableView_Edges->selectionModel()->selectedRows();
    if(selected_row.empty()){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"Selected row is empty["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    QVector<GeneralMatrixType> matrices;
    GeneralMatrixType m;
    for(size_t i=0;i<selected_row.size();i++){
        GraphHelper::getInformationMatrix(edge_table->at(selected_row[i].row()), m);
        matrices.push_back(m);
    }
    ui->widget_MatrixManipulator->setMatrices(matrices);
}


void EdgeManipulator::on_pushButton_RobustKernel_Apply_clicked()
{
//    DEBUG_FUNC_STAMP;
    QModelIndexList selected_row = ui->tableView_Edges->selectionModel()->selectedRows();
    if(selected_row.empty()){
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
        std::cerr<<"Selected row is empty["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    QList<EdgeModifyAction::Ptr> modifications;
    auto kernel_name = ui->comboBox_RobustKernel->currentText();
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
    std::cerr<<"Selected kernel is "<<kernel_name.toStdString()<<"["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
    if(kernel_name.compare("None")==0){
        for(size_t i=0;i<selected_row.size();i++){
            modifications.push_back(EdgeModifyAction::Ptr(
                                        new EdgeModifySetRobustKernel(edge_table->at(selected_row[i].row()),nullptr)));
        }
    }else{
        g2o::AbstractRobustKernelCreator* creator
                = g2o::RobustKernelFactory::instance()->creator(kernel_name.toStdString());
        double kernel_width = ui->doubleSpinBox_KernelWidth->value();
        for(size_t i=0;i<selected_row.size();i++){
            g2o::RobustKernel* kernel = creator->construct();
            kernel->setDelta(kernel_width);
            modifications.push_back(EdgeModifyAction::Ptr(
                                        new EdgeModifySetRobustKernel(edge_table->at(selected_row[i].row()), kernel)));
        }
#ifdef EDGE_MANIPULATOR_CONSOLE_DEBUG
    std::cerr<<"Kernel width : "<<kernel_width<<"["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
    }
    Q_EMIT edgesShouldBeModified(modifications);
}
