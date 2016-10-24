#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QDir>
#include "Global.h"
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <QTableView>
#include "icpdialog.h"
#include "QGLHelper.h"
#include <g2o/core/optimization_algorithm_factory.h>


#define MAIN_WINDOW_CONSOLE_DEBUG
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
    //    //    QObject::connect(this->ui->pushButton_ManualLoopClosing, SIGNAL(clicked()),
    //    //                     this->ui->widget_CloudViewer, SLOT(slot_manualLoopClosing()));
    QObject::connect(this, SIGNAL(loopClosingAdded(int,int,PosTypes::Pose3D,g2o::EdgeSE3::InformationType)),
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
    int n_iter = ui->spinBox_OptimizationIterations->value();
    if(!graph_slam.optimize(n_iter)){
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

void MainWindow::on_action_Edges_Table_triggered()
{
    if(graph_table_dialog){
        return;
    }
#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Opening Edge Table Dialog");
#endif

    logger->logMessage("Opening Edge Table Dialog");
    graph_table_dialog = new GraphTableDialog(this);
    graph_table_dialog->setModal(false);

#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Setting up Signal-Slot connections");
#endif

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

#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Updating graph table");
#endif

    graph_slam.slot_sendGraphTable();

#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    DEBUG_MESSAGE_WITH_FUNC_INFO("Start showing graph table dialog.");
#endif
    graph_table_dialog->show();
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
    QObject::disconnect(graph_table_dialog, SIGNAL(edgesShouldBeModified(EdgeModifications)),
                        &graph_slam, SLOT(modifyEdges(EdgeModifications)));
    this->graph_table_dialog = NULL;
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

void generateClusters(const QList<int>& input,  QVector<QVector<int>>& clusters){
    auto sel = input;
    qSort(sel.begin(), sel.end());
    int prev = -1000;
    for(const auto&a:sel){
        if(a-prev > 5){
            clusters.push_back(QVector<int>());
        }
        clusters.back().push_back(a);
        prev = a;
    }
#ifdef USE_CONSOLE_DEBUG
    std::cerr<<clusters.size()<<" clusters are generated."<<std::endl;
#endif
}

void MainWindow::on_pushButton_ManualLoopClosing_clicked()
{
    auto selections = ui->widget_CloudViewer->getSelections();
    if(selections.size() < 2){
        logger->logMessage("Please, choose at least 2 vertices.");
        return;
    }
    QVector<QVector<int>> clusters;
    generateClusters(selections, clusters);
    if(clusters.size() != 2){
        logger->logMessage(QString("2 clusters are needed.(%1 generated)").arg(clusters.size()).toStdString().c_str());
        return;
    }
    if(ui->checkBox_WRT_OldestVertex->isChecked())
        qSort(clusters[0].begin(), clusters[0].end());
    else
        qSort(clusters[0].begin(), clusters[0].end(),[](const int& v1, const int& v2){return v1>v2;});
    if(ui->checkBox_WRT_OldestVertex2->isChecked())
        qSort(clusters[1].begin(), clusters[1].end());
    else
        qSort(clusters[1].begin(), clusters[1].end(),[](const int& v1, const int& v2){return v1>v2;});
    auto cloud_model = graph_slam.getCompositedPointCloudDisplayer(clusters[0]);
    auto cloud_template = graph_slam.getCompositedPointCloudDisplayer(clusters[1]);

    ICPDialog icp_dialog;
    icp_dialog.setModelCloud(cloud_model);
    icp_dialog.setTemplateCloud(cloud_template);
    icp_dialog.initialize();
    icp_dialog.setModal(true);
    if(icp_dialog.exec()){
        qglviewer::Frame icp_result =  icp_dialog.getICPResult();
        std::ostringstream oss;
        oss<<"ICP Result : \n"<<std::endl;
        oss<<"\tPosition : "<<icp_result.position()<<std::endl;
        oss<<"\tOrientation : "<<icp_result.orientation()<<std::endl;
        logger->logMessage(oss.str().c_str());
        g2o::EdgeSE3::InformationType info;
        info.setIdentity();
        graph_slam.addLoopClosing(clusters[0][0], clusters[1][0],
                QGLHelper::toPosTypesPose3D(icp_result), info);
    }
}

void MainWindow::on_action2D_Project_triggered()
{
    auto tmp = ui->widget_CloudViewer->getSelections();
    if(tmp.empty()){
        return;
    }
#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    std::cerr<<tmp.size()<<" vertices will be converted into a map"<<std::endl;
#endif
    QVector<int> selected;
    for(auto i:tmp){
        selected.push_back(i);
    }
    qSort(selected);

    QVector<PointCloudDisplayer::Ptr> clouds;
    graph_slam.getPointCloudDisplayers(selected, clouds);

    qglviewer::Frame origin;
    origin.setOrientation(0,0,0,1);
    origin.setPosition(clouds[0]->frame.position());

    double max_x=-1000000000, max_y=-100000000, min_x=100000000000, min_y=10000000000;
    for(int i=0;i<clouds.size();i++){
        PointCloudDisplayer::Ptr cloud_ptr = clouds[i];
        TransformPointCloudDisplayer(*cloud_ptr, origin);
        int npts = cloud_ptr->nPoints();
        int nf = cloud_ptr->fields.size();
        for(size_t pi=0;pi<npts;pi++){
            size_t idx = pi * nf;
            if(cloud_ptr->data[idx] >max_x){
                max_x = cloud_ptr->data[idx];
            }
            if(cloud_ptr->data[idx] < min_x){
                min_x = cloud_ptr->data[idx];
            }
            if(cloud_ptr->data[idx+1] > max_y){
                max_y = cloud_ptr->data[idx+1];
            }
            if(cloud_ptr->data[idx+1] < min_y){
                min_y = cloud_ptr->data[idx+1];
            }
        }
    }
    double res = 0.10;
    max_x += res*5; max_y += res*5;
    min_x -= res*5; min_y -= res*5;

    size_t width = (max_x - min_x) / res;
    size_t height = (max_y-min_y)  / res;
    std::vector<std::vector<double>> grid_map;
    std::vector<std::vector<size_t>> cell_count;
#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    std::cerr<<"[minx, miny, maxx, maxy] = ["<<min_x<<", "<<min_y<<"," <<max_x<<", "<<max_y<<"]"<<std::endl;
    std::cerr<<"Map dimension : "<<height<<" X "<<width<<std::endl;
#endif

    grid_map.resize(height);
    cell_count.resize(height);
    for(size_t i=0;i<height;i++){
        grid_map[i].resize(width, 0.0);
        cell_count[i].resize(width,0);
    }
    for(int i=0;i<clouds.size();i++){
        PointCloudDisplayer::Ptr cloud_ptr = clouds[i];
        int npts = cloud_ptr->nPoints();
        int nf = cloud_ptr->fields.size();

        for(int pi=0;pi<npts;pi++){
            size_t idx = pi * nf;
            size_t xi = static_cast<size_t>((cloud_ptr->data[idx] - min_x)/res);
            size_t yi = static_cast<size_t>((cloud_ptr->data[idx+1] - min_y)/res);
            size_t n = cell_count[yi][xi];
            double weight = 1.0*n/(n+1);
            grid_map[yi][xi] =grid_map[yi][xi]*weight+ cloud_ptr->data[idx+3]/(n+1);
            cell_count[yi][xi] += 1;
        }
    }

#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    std::cerr<<"Writing file..."<<std::endl;
#endif
    QString file_name = QDir::home().path()+"/manual_slam_intensity_map.pgm";
    std::ofstream file(file_name.toStdString().c_str(), std::ios_base::out|std::ios_base::binary|std::ios_base::trunc);
    file<<"P5\n"<<width<<" "<<height<<"\n"<<255<<"\n";
    for(size_t iy=height;iy>0;iy--){
        for(size_t ix=0;ix<width;ix++){
//            std::cerr<<grid_map[iy][ix]<<" ";
            file<<static_cast<unsigned char>(grid_map[iy-1][ix]);
        }
//        std::cerr<<std::endl;
    }
    file<<std::flush;
#ifdef MAIN_WINDOW_CONSOLE_DEBUG
    std::cerr<<"Done!"<<std::endl;
#endif
}
