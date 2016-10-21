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
    QObject::connect(&graph_slam, SIGNAL(graphDisplayUpdated(GraphDisplayer::Ptr)),
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
    QObject::connect(ui->widget_CloudViewer, SIGNAL(verticesSelected(QList<int>)),
                        this, SLOT(selectVertices(QList<int>)));
    QObject::connect(&timer, SIGNAL(timeout()), this, SLOT(slot_autoSave()));
    timer.setInterval(1000*60*5);//5 mintue auto save.
    timer.start();
}

void MainWindow::loadGraphFromPCDDatabase(const QDir &db_dir)
{
//    db_dir.entryList({"*.pcd"},QDir::Files, QDir::Name);
//    QString motion(db_dir.path()+"/motion.txt");

//    if(!QFile(motion).exists()){
//        throw PathNotExist(motion.toStdString());
//    }
//    graph_slam.reset();
//    std::ifstream file(motion.toStdString());
//    std::string token;
//    int id1, id2;
//    double dx, dy, dz, qw, qx, qy, qz;
//    GraphSLAM::InformationType info = GraphSLAM::InformationType::Identity();
//    info(0,0) = info(1,1)  = 100.0;
//    info(2,2) = 100.0;

//    info(3,3) =1000.0;
//    info(4,4) =1000.0;
//    info(5,5) =50;

//    size_t cnt = 0;
//    for(std::string line; std::getline(file, line);cnt++){
//        std::stringstream str(line);
//        str >> id1;        str >> id2;
//        str >> dx;         str >> dy;        str >> dz;
//        str >> qx;        str >> qy;        str >> qz;        str >> qw;

//        //set positional data.
//        PosTypes::Pose3D pos(dx, dy, dz, qx, qy, qz, qw);

//        //read sensor data(Point Cloud)
//        GraphSLAM::ScanDataType::Ptr cloud(new GraphSLAM::ScanDataType());
//        std::ostringstream oss;
//        oss<<db_dir.path().toStdString()<<"/"<<std::setfill('0')<<std::setw(10)<<id2<<".pcd";
//        std::string pcd_name = oss.str();

//        if(pcl::io::loadPCDFile<GraphSLAM::ScanDataType::PointType>(pcd_name, *cloud)<0){
//            logger->logMessage((std::string("Failed to read ")+pcd_name).c_str());
//            cloud.reset();
//        }
//        //on a reading failure, cloud(shared_ptr) will have NULL pointer

//        graph_slam.addVertex(pos, cloud, true);
//        if(id1 == -1){
//            graph_slam.setVertexFixed(0, true);
//        }else{
//            int n = graph_slam.nVertices();
//            graph_slam.addEdgeFromVertices(n-2, n-1, info);
//        }
//    }
}

void MainWindow::resetMessage()
{
    ui->plainTextEdit_Message->clear();
}

void MainWindow::appendMessage(QString msg)
{
    ui->plainTextEdit_Message->appendPlainText(msg);
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
    this->graph_slam.reset();
}

void MainWindow::on_pushButton_ClearLoopClosings_clicked()
{
    auto reply= QMessageBox::question(this, "Clearing Loop Closing",
                                  "Do you want to remove all loop closures?",
                                  QMessageBox::Yes|QMessageBox::No);
    if(reply == QMessageBox::Yes){
        graph_slam.resetLoopClosings();
    }
}

void MainWindow::on_action_Save_As_G2O_triggered()
{
    //    QFileDialog file_dlg(this, "Open Database","");
    //    file_dlg.setFileMode(QFileDialog::DirectoryOnly);
    //    file_dlg.setModal(true);
    //    if(file_dlg.exec()){
    //        QDir db = file_dlg.directory();
      QDir db(QDir::homePath()+"/manual_slam_db");
    QString file_name = QFileDialog::getSaveFileName(this, "Save as", db.path(), "*.g2o");
    if(file_name.size()){
        logger->logMessage((QString("Saving pose graph in ")+file_name).toStdString().c_str());
        if(!graph_slam.saveAsG2O(file_name)){
            logger->logMessage("Saving g2o failed.");
        }
    }
}

void MainWindow::slot_graphTableDialogClosed()
{
    logger->logMessage("Graph Edge Dialog closed");
    QObject::disconnect(graph_table_dialog, SIGNAL(destroyed()),
                        this,SLOT(slot_graphTableDialogClosed()));
    QObject::disconnect(graph_table_dialog, SIGNAL(edgesShouldBeRemoved(QList<const g2o::EdgeSE3*>)),
                        &graph_slam, SLOT(removeEdges(QList<const g2o::EdgeSE3*>)));
    QObject::disconnect(&graph_slam, SIGNAL(graphTableUpdated(GraphTableData*)),
                        graph_table_dialog, SLOT(setGraphTable(GraphTableData*)));
    QObject::disconnect(&graph_slam, SIGNAL(edgesSelected(QList<int>,QList<int>)),
                     graph_table_dialog, SLOT(selectEdges(QList<int>,QList<int>)));

    this->graph_table_dialog = NULL;
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
                     this,SLOT(slot_graphTableDialogClosed()));
    QObject::connect(graph_table_dialog, SIGNAL(edgesShouldBeRemoved(QList<const g2o::EdgeSE3*>)),
                     &graph_slam, SLOT(removeEdges(QList<const g2o::EdgeSE3*>)));
    QObject::connect(&graph_slam, SIGNAL(graphTableUpdated(GraphTableData*)),
                     graph_table_dialog, SLOT(setGraphTable(GraphTableData*)));
    QObject::connect(&graph_slam, SIGNAL(edgesSelected(QList<int>,QList<int>)),
                     graph_table_dialog, SLOT(selectEdges(QList<int>,QList<int>)));
    QObject::connect(graph_table_dialog, SIGNAL(edgesShouldBeModified(EdgeModifications)),
                     &graph_slam, SLOT(modifyEdges(EdgeModifications)));

    graph_slam.slot_sendGraphTable();
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

void MainWindow::selectVertices(QList<int> vertices_id)
{
    graph_slam.slot_selectVertices(vertices_id, true);
}

void MainWindow::on_pushButton_ClearSelection_clicked()
{
    this->ui->widget_CloudViewer->slot_clearSelections();
}

void MainWindow::on_pushButton_SetSelectedVerticesVisible_clicked()
{
    ui->widget_CloudViewer->slot_setSelectedCloudsVisible();
    ui->widget_CloudViewer->slot_clearSelections();
}

void MainWindow::on_pushButton_SetVisibleAll_clicked()
{
    ui->widget_CloudViewer->slot_clearVisibleFlags(true);
}

void MainWindow::on_checkBox_ShowPointCloud_toggled(bool checked)
{
    ui->widget_CloudViewer->slot_setCloudsVisible(checked);
}

void MainWindow::slot_autoSave()
{
    if(!graph_slam.initialized()){
        std::cerr<<"graph is not initialized.["<<Q_FUNC_INFO<<"]"<<std::endl;
        return;
    }
    QDir dir(QDir::homePath()+"/.manual_slam_autosave");
    if(! dir.exists()){
        logger->logMessage("Creating directory for auto-saving.");
        dir.mkpath(dir.path());
    }
    QString file_name = dir.path() + "/auto_save.g2o";
    graph_slam.saveAsG2O(file_name);
    QString msg = "auto save to "+file_name;
    logger->logMessage(msg.toStdString().c_str());
}
