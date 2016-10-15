#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QDir>
#include "Global.h"
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
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

    pcl::PCDReader reader;
    size_t cnt = 0;
    for(std::string line; std::getline(file, line);cnt++){
        if(cnt >= 20){
            break;
        }
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

void MainWindow::resetMessage()
{
    ui->plainTextEdit_Message->clear();
}

void MainWindow::appendMessage(QString msg)
{
    ui->plainTextEdit_Message->appendPlainText(msg+"\n");
}

void MainWindow::on_action_File_Open_triggered()
{
    //    QFileDialog file_dlg(this, "Open Database","");
    //    file_dlg.setFileMode(QFileDialog::DirectoryOnly);
    //    file_dlg.setModal(true);
    //    if(file_dlg.exec()){
    //        QDir db = file_dlg.directory();
    QDir db("/home/ub1404/Incheon");
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

}

void MainWindow::on_pushButton_Reset_clicked()
{

}

void MainWindow::on_pushButton_SearchLoopClosings_clicked()
{

}
