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



GraphSLAM::GraphSLAM()
{
    reset();
    init();
}

void GraphSLAM::init()
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

void GraphSLAM::optimize(const int &n_iterations)
{

}

bool GraphSLAM::loadFromG2OFile(const std::string &file_name)
{
    std::ifstream file(file_name);

    logger->logMessage((std::string("Start loading from file(")+file_name+std::string(")")).c_str());
    if(!this->m_g2o_graph->load(file)){
        return false;
    }
    init();
    return true;
}

bool GraphSLAM::setVertexFixed(const int &id, const bool &fixed)
{
    if(id < m_vertices.size()){
        m_vertices[id]->setFixed(fixed);
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

void GraphSLAM::sendGraph()
{
    GraphDisplayer::Ptr graph_disp(new GraphDisplayer());
    graph_disp->m_vertices.resize(m_vertices.size());
    for(size_t i=0; i<m_vertices.size();i++){
        GraphHelper::convertSE3To4x4Matrix(
                    m_vertices[i]->estimate(),
                    &(graph_disp->m_vertices[i].m[0]));
    }

    graph_disp->m_loop_edges.reserve(m_loop_edges.size());
    for(size_t i=0; i<m_loop_edges.size();i++){
        graph_disp->m_loop_edges.push_back(
                    GraphDisplayer::EdgeDisplayType(
                        m_loop_edges[i]->vertex(0)->id(),
                        m_loop_edges[i]->vertex(1)->id()));
    }

    graph_disp->m_motion_edges.reserve(m_motion_edges.size());
    for(size_t i=0; i<m_motion_edges.size();i++){
        graph_disp->m_motion_edges.push_back(
                    GraphDisplayer::EdgeDisplayType(
                        m_motion_edges[i]->vertex(0)->id(),
                        m_motion_edges[i]->vertex(1)->id()));
    }

    Q_EMIT graphUpdated(graph_disp);
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

void GraphSLAM::slot_searchLoopClosings(std::vector<int> idx_old, std::vector<int> idx_new, PosTypes::Pose3D tr_new)
{

}

void GraphSLAM::slot_startOptimize(int max_iterations)
{

}

void GraphSLAM::searchLoopClosing(const std::vector<int> &idx_old, const std::vector<int> &idx_new, const PosTypes::Pose3D &tr_new)
{

}



