#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QDir>
#include "Global.h"
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <QTableView>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    graph_table_dialog(NULL)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init()
{
    QObject::connect(&graph_slam, SIGNAL(showMessage(QString)), this, SLOT(appendMessage(QString)));
    QObject::connect(&graph_slam, SIGNAL(graphUpdated(GraphDisplayer::Ptr)),
                     ui->widget_CloudViewer, SLOT(setGraphDisplayer(GraphDisplayer::Ptr)));
    QObject::connect(&graph_slam, SIGNAL(pointCloudsUpdated(QVector<PointCloudDisplayer::Ptr>)),
                     ui->widget_CloudViewer, SLOT(setPointCloudDisplayers(QVector<PointCloudDisplayer::Ptr>)));
    QObject::connect(this->ui->pushButton_ManualLoopClosing, SIGNAL(clicked()),
                     this->ui->widget_CloudViewer, SLOT(slot_manualLoopClosing()));
    QObject::connect(this->ui->widget_CloudViewer, SIGNAL(loopClosingAdded(int,int,PosTypes::Pose3D,g2o::EdgeSE3::InformationType)),
                     &graph_slam, SLOT(addLoopClosing(int,int,PosTypes::Pose3D,g2o::EdgeSE3::InformationType)));
    QObject::connect(this->ui->doubleSpinBox_AlphaPointCloud, SIGNAL(valueChanged(double)),
                     this->ui->widget_CloudViewer, SLOT(slot_setPointAlpha(double)));
    QObject::connect(this->ui->doubleSpinBox_PointSize, SIGNAL(valueChanged(double)),
                     this->ui->widget_CloudViewer, SLOT(slot_setPointSize(double)));
    QObject::connect(&graph_slam, SIGNAL(graphUpdated2(EdgeTableModel::Ptr)),
                     this, SLOT(slot_updateGraph(EdgeTableModel::Ptr)));

    loop_table_model.reset(new EdgeTableModel());
    this->ui->tableView_LoopEdges->setModel(loop_table_model.get());
    ui->tableView_LoopEdges->setColumnWidth(3, 150);
}

void MainWindow::loadGraphFromPCDDatabase(const QDir &db_dir)
{
    db_dir.entryList({"*.pcd"},QDir::Files, QDir::Name);
    QString motion(db_dir.path()+"/motion.txt");

    if(!QFile(motion).exists()){
        throw PathNotExist(motion.toStdString());
    }
    graph_slam.reset();
    std::ifstream file(motion.toStdString());
    std::string token;
    int id1, id2;
    double dx, dy, dz, qw, qx, qy, qz;
    GraphSLAM::InformationType info = GraphSLAM::InformationType::Identity();
    info(0,0) = info(1,1)  = 100.0;
    info(2,2) = 100.0;

    info(3,3) =1000.0;
    info(4,4) =1000.0;
    info(5,5) =50;

    size_t cnt = 0;
    for(std::string line; std::getline(file, line);cnt++){
        std::stringstream str(line);
        str >> id1;        str >> id2;
        str >> dx;         str >> dy;        str >> dz;
        str >> qx;        str >> qy;        str >> qz;        str >> qw;

        //set positional data.
        PosTypes::Pose3D pos(dx, dy, dz, qx, qy, qz, qw);

        //read sensor data(Point Cloud)
        GraphSLAM::ScanDataType::Ptr cloud(new GraphSLAM::ScanDataType());
        std::ostringstream oss;
        oss<<db_dir.path().toStdString()<<"/"<<std::setfill('0')<<std::setw(10)<<id2<<".pcd";
        std::string pcd_name = oss.str();

        if(pcl::io::loadPCDFile<GraphSLAM::ScanDataType::PointType>(pcd_name, *cloud)<0){
            logger->logMessage((std::string("Failed to read ")+pcd_name).c_str());
            cloud.reset();
        }
        //on a reading failure, cloud(shared_ptr) will have NULL pointer

        graph_slam.addVertex(pos, cloud, true);
        if(id1 == -1){
            graph_slam.setVertexFixed(0, true);
        }else{
            int n = graph_slam.nVertices();
            graph_slam.addEdgeFromVertices(n-2, n-1, info);
        }
    }
}

void MainWindow::loadGraphFromG2ODatabase(const QDir &db_dir)
{

}

void MainWindow::resetMessage()
{
    ui->plainTextEdit_Message->clear();
}

void MainWindow::appendMessage(QString msg)
{
    ui->plainTextEdit_Message->appendPlainText(msg);
}

void MainWindow::setSelections(GraphSelectionInfo info)
{
    this->selection_info = info;
}

void MainWindow::slot_updateGraph(EdgeTableModel::Ptr graph){
    ui->tableView_LoopEdges->setModel(graph.get());
    loop_table_model = graph;//To prevent shared_ptr destroy the item.
}

void MainWindow::slot_edgesTableDialogClosed()
{
    logger->logMessage("Graph Edge Dialog closed");
    QObject::disconnect(graph_table_dialog, SIGNAL(destroyed()),
                     this,SLOT(slot_edgesTableDialogClosed()));
    this->graph_table_dialog = NULL;
}

void MainWindow::on_action_File_Open_triggered()
{
    //    QFileDialog file_dlg(this, "Open Database","");
    //    file_dlg.setFileMode(QFileDialog::DirectoryOnly);
    //    file_dlg.setModal(true);
    //    if(file_dlg.exec()){
    //        QDir db = file_dlg.directory();

    QDir db(QDir::homePath()+"/manual_slam_db");
    try{
        loadGraphFromPCDDatabase(db);
    }
    catch(PathNotExist& e){
        logger->logMessage(e.what());
        return;
    }
    std::ostringstream oss;
    oss<<graph_slam.nVertices()<<" vertices and "<<graph_slam.nEdges()<<" edges are lodaed.";
    logger->logMessage(oss.str().c_str());
    graph_slam.sendGraph();
    graph_slam.sendPointCloud();
    //    }
}

void MainWindow::on_pushButton_Optimize_clicked()
{
    graph_slam.init();
    if(!graph_slam.optimize(100)){
        return;
    }
    graph_slam.sendGraph();
    graph_slam.sendPointCloud();
}

void MainWindow::on_pushButton_Reset_clicked()
{

}

void MainWindow::on_pushButton_ClearLoopClosings_clicked()
{
    graph_slam.resetLoopClosings();
}

void MainWindow::on_pushButton_ApplyInfoMatrix_clicked()
{

}

void MainWindow::on_pushButton_DeleteLoopEdge_clicked()
{

}

void MainWindow::on_pushButton_DeleteSelectedLoopClosings_clicked()
{
    QModelIndexList selected_row = ui->tableView_LoopEdges->selectionModel()->selectedRows();
    if(selected_row.empty()){
        return;
    }
    QList<g2o::EdgeSE3*> edges;
    for(size_t i=0;i<selected_row.size();i++){
        //        std::cerr<<selected_row[i].row()<<std::endl;
        edges.push_back(this->loop_table_model->at(selected_row[i].row()));
    }
    graph_slam.removeEdges(edges);
}

void MainWindow::on_action_Save_As_G2O_triggered()
{
    logger->logMessage("Saving as g2o");
    //TODO
}

void MainWindow::on_action_Edges_Table_triggered()
{
    if(graph_table_dialog){
        return;
    }
     logger->logMessage("Opening Edge Table Dialog");
    graph_table_dialog = new GraphTableDialog(this);
    graph_table_dialog->setModal(false);
    QObject::connect(graph_table_dialog, SIGNAL(destroyed()),
                     this,SLOT(slot_edgesTableDialogClosed()));
    graph_table_dialog->show();
}

void MainWindow::on_action_Open_From_G2O_triggered()
{
    QDir db(QDir::homePath()+"/manual_slam_db");
    try{
        graph_slam.loadFromG2ODB(db);
    }
    catch(PathNotExist& e){
        logger->logMessage(e.what());
        return;
    }
    catch(std::runtime_error& e){
        logger->logMessage(e.what());
        return;
    }

    std::ostringstream oss;
    oss<<graph_slam.nVertices()<<" vertices and "<<graph_slam.nEdges()<<" edges are lodaed.";
    logger->logMessage(oss.str().c_str());
    graph_slam.sendGraph();
    graph_slam.sendPointCloud();
}
