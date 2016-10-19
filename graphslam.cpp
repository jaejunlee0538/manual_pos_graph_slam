#include "graphslam.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/block_solver.h>
#include <Global.h>
#include <string>
#include <pcl/PCLPointField.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <QDir>
#include "exceptions.h"
#include <g2o/core/factory.h>
#define GRAPH_SLAM_DEBUG
namespace __private{
g2o::RegisterTypeProxy<g2o::VertexSE3> register_vertex_se3("VERTEX_SE3:QUAT");
g2o::RegisterTypeProxy<g2o::EdgeSE3> register_edge_se3("EDGE_SE3:QUAT");
}
GraphSLAM::GraphSLAM()
{
    reset();
    init();
}

void GraphSLAM::init()
{
    this->initG2OGraph();
}

void GraphSLAM::reset()
{
    if(!m_g2o_graph){
        m_g2o_graph.reset(new g2o::SparseOptimizer());
    }
    m_g2o_graph->clear();
    m_loop_edges.clear();
    m_motion_edges.clear();
    m_vertices.clear();
    m_scan_data.clear();
}

void GraphSLAM::resetLoopClosings()
{
    for(size_t i=0;i<m_loop_edges.size();i++){
        m_g2o_graph->removeEdge(m_loop_edges[i]);
    }
    m_loop_edges.clear();
}

bool GraphSLAM::optimize(const int &n_iterations)
{
    logger->logMessage("Start graph optimization. Please wait...");
    if(!m_g2o_graph->initializeOptimization()){
        logger->logMessage("Initialization failed.");
        return false;
    }
    m_g2o_graph->optimize(n_iterations);
    logger->logMessage("Optimization done.");
    return true;
}

void GraphSLAM::loadFromG2ODB(const QDir &db_dir)
{

    reset();

    QString g2o_file = db_dir.path()+"/motion.g2o";
    if(!QFile::exists(g2o_file)){
        throw PathNotExist(g2o_file.toStdString());
    }

    std::ifstream file(g2o_file.toStdString());
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"Loading g2o."<<std::endl;
#endif
    if(!this->m_g2o_graph->load(file)){
        throw std::runtime_error("G2O load failed");
    }

#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"Copying vertices"<<std::endl;
#endif
    //WARN : assuming that vertex id starts from '0' and ends at 'vertices().size()-1'.
    size_t n_vertex = m_g2o_graph->vertices().size();
    for(size_t id=0;id<n_vertex;id++){
        g2o::VertexSE3* v_se3 = dynamic_cast<g2o::VertexSE3*>(m_g2o_graph->vertex(id));
        if(v_se3 == NULL){
            throw std::runtime_error("dynamic_cast<g2o::VertexSE3*> failed");
        }
        m_vertices.push_back(v_se3);
    }
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"Copying edges"<<std::endl;
#endif
    for(const auto& ev:m_g2o_graph->edges()){
        g2o::EdgeSE3* e_se3 = dynamic_cast<g2o::EdgeSE3*>(ev);
        if(e_se3 == NULL){
            throw std::runtime_error(" dynamic_cast<g2o::EdgeSE3*> failed.");
        }

        if(abs(e_se3->vertex(1)->id() - e_se3->vertex(0)->id()) > 1){
            m_loop_edges.push_back(e_se3);
        }else{
            m_loop_edges.push_back(e_se3);
        }
    }

    for(const auto& pv:m_vertices){
        ScanDataType::Ptr cloud(new ScanDataType());
        std::string pcd_file = QString(db_dir.path()+"/%1.pcd").arg(pv->id(), 10,10,QChar('0')).toStdString();
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"Reading "<<pcd_file<<std::endl;
#endif
        if(pcl::io::loadPCDFile<ScanDataType::PointType>(pcd_file,*cloud)<0){
            logger->logMessage((std::string("Failed to read ")+pcd_file).c_str());
            cloud.reset();
        }
        m_scan_data.push_back(cloud);
    }
}

bool GraphSLAM::setVertexFixed(const int &id, const bool &fixed)
{
    if(id < m_vertices.size()){
        m_vertices[id]->setFixed(fixed);
        logger->logMessage(QString("Set vertex(%1) fixed.").arg(id).toStdString().c_str());
    }else{
        logger->logMessage("Index is out of range(GraphSLAM::setVertexFixed)");
    }
}


bool GraphSLAM::addVertex(const PosTypes::Pose3D &p, ScanDataType::Ptr cloud_ptr, const bool &incremental)
{
    VertexType * v;
    int sz_vertices = m_vertices.size();
    if(incremental && sz_vertices > 0){
        v = GraphHelper::createVertexSE3(sz_vertices, m_vertices[sz_vertices-1], p);
    }else{
        v = GraphHelper::createVertexSE3(sz_vertices, p);
    }

    m_g2o_graph->addVertex(v);
    m_vertices.push_back(v);
    m_scan_data.push_back(cloud_ptr);
    return true;
}

bool GraphSLAM::addEdgeFromVertices(const int &vi, const int &vj,
                                    const g2o::EdgeSE3::InformationType &info_mat)
{
    if(vi==vj || vi < 0 || vj < 0 || vi>=m_vertices.size() || vj>= m_vertices.size()){
        logger->logMessage("Index is out of range(GraphSLAM::addEdgeFromVertices)");
        return false;
    }

    EdgeType * e = GraphHelper::createEdgeSE3FromVertices(m_g2o_graph->edges().size(),
                                                          m_vertices[vi], m_vertices[vj],
                                                          info_mat);
    m_g2o_graph->addEdge(e);
    if(abs(vi - vj) == 1){
        m_motion_edges.push_back(e);
    }else{
        m_loop_edges.push_back(e);
    }
    return true;
}

bool GraphSLAM::addLoopClosing(const int &vi, const int &vj,
                               const PosTypes::Pose3D &data, const g2o::EdgeSE3::InformationType &info_mat)
{
    EdgeType* e = GraphHelper::createEdgeSE3(m_g2o_graph->edges().size(), m_vertices[vi], m_vertices[vj], data, info_mat);
    m_g2o_graph->addEdge(e);
    if(abs(vi - vj) == 1){
        throw std::runtime_error("This is not a loop closing edge.");
    }else{
        m_loop_edges.push_back(e);
    }
    this->sendGraph();
    return true;
}

void GraphSLAM::removeEdge(g2o::EdgeSE3 *e)
{
    m_g2o_graph->removeEdge(e);
    m_loop_edges.removeOne(e);
    this->sendGraph();
}

void GraphSLAM::removeEdges(QList<g2o::EdgeSE3 *> es)
{
    for(size_t i=0;i<es.size();i++){
        m_g2o_graph->removeEdge(es[i]);
        m_loop_edges.removeOne(es[i]);
    }
    this->sendGraph();
}

void GraphSLAM::sendGraph()
{
    GraphDisplayer::Ptr graph_disp(new GraphDisplayer());

    graph_disp->m_vertices.resize(m_vertices.size());
    for(size_t i=0; i<m_vertices.size();i++){
        GraphHelper::convertSE3To4x4Matrix(
                    m_vertices[i]->estimate(),
                    &(graph_disp->m_vertices[i].m[0]));
    }

    EdgeTableModel::Ptr loop_edge_table(new EdgeTableModel());
    graph_disp->m_loop_edges.reserve(m_loop_edges.size());
    for(size_t i=0; i<m_loop_edges.size();i++){
        graph_disp->m_loop_edges.push_back(
                    GraphDisplayer::EdgeDisplayType(
                        m_loop_edges[i]->vertex(0)->id(),
                        m_loop_edges[i]->vertex(1)->id()));
        loop_edge_table->pushBack(m_loop_edges[i]);
    }

    graph_disp->m_motion_edges.reserve(m_motion_edges.size());
    for(size_t i=0; i<m_motion_edges.size();i++){
        graph_disp->m_motion_edges.push_back(
                    GraphDisplayer::EdgeDisplayType(
                        m_motion_edges[i]->vertex(0)->id(),
                        m_motion_edges[i]->vertex(1)->id()));
    }

    Q_EMIT graphUpdated(graph_disp);
    Q_EMIT graphUpdated2(loop_edge_table);
}

void GraphSLAM::sendPointCloud()
{
    QVector<PointCloudDisplayer::Ptr> cloud_out;
    cloud_out.resize(m_vertices.size());
    for(size_t i=0; i<m_vertices.size();i++){
        cloud_out[i].reset(new PointCloudDisplayer());
        GraphHelper::convertSE3ToqglviewerFrame(m_vertices[i]->estimate(), cloud_out[i]->frame);
        if(m_scan_data[i]){
            cloud_out[i]->fields = {"x","y","z","intensity"};
            cloud_out[i]->start = 0;
            size_t n_pts = m_scan_data[i]->height*m_scan_data[i]->width;
            cloud_out[i]->data.resize(4*n_pts);
            size_t n_valid = 0;
            for(size_t k=0;k<n_pts;k++){
                if(pcl::isFinite(m_scan_data[i]->points[k])){
                    size_t idx = n_valid * 4;
                    cloud_out[i]->data[idx] = m_scan_data[i]->points[k].x;
                    cloud_out[i]->data[idx+1] = m_scan_data[i]->points[k].y;
                    cloud_out[i]->data[idx+2] = m_scan_data[i]->points[k].z;
                    cloud_out[i]->data[idx+3] = m_scan_data[i]->points[k].intensity;
                    n_valid++;
                }
            }
            cloud_out[i]->data.resize(n_valid*4);
        }
    }
    Q_EMIT pointCloudsUpdated(cloud_out);
}

void GraphSLAM::slot_searchLoopClosings(QVector<int> idx_old, QVector<int> idx_new,
                                        PosTypes::Pose3D tr_new)
{

}

void GraphSLAM::slot_startOptimize(int max_iterations)
{

}

void GraphSLAM::loadPCDFiles()
{

}

void GraphSLAM::initG2OGraph()
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1> > SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmGaussNewton* solverGauss =
            new g2o::OptimizationAlgorithmGaussNewton(blockSolver);
    this->m_g2o_graph->setAlgorithm(solverGauss);
    logger->logMessage("Optimization algorithm loaded");
}


