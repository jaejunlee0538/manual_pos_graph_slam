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
#include <map>
#include "graphhelper.h"

#define GRAPH_SLAM_DEBUG

namespace __private{
//required for loading graph from g2o file.
//Each Vertex/Edge types must be registered in g2o::Factory(singleton) beforehand.
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
    this->sendGraph();
}

void GraphSLAM::resetLoopClosings()
{
    for(size_t i=0;i<m_loop_edges.size();i++){
        m_g2o_graph->removeEdge(m_loop_edges[i]);
    }
    m_loop_edges.clear();
    this->sendGraph();
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

    //the number of motion edges is determined by the number of vertices.
    m_motion_edges.resize(m_vertices.size()-1);
    for(const auto& ev:m_g2o_graph->edges()){
        g2o::EdgeSE3* e_se3 = dynamic_cast<g2o::EdgeSE3*>(ev);
        if(e_se3 == NULL){
            throw std::runtime_error(" dynamic_cast<g2o::EdgeSE3*> failed.");
        }

        if(abs(e_se3->vertex(1)->id() - e_se3->vertex(0)->id()) > 1){
            m_loop_edges.push_back(e_se3);
        }else{
            int v_id = e_se3->vertex(0)->id();
            if(v_id >= m_motion_edges.size()){
                throw std::runtime_error("Invliad motion edge.");
            }
            m_motion_edges[v_id] = e_se3;
            //            m_motion_edges.push_back(e_se3);
        }
    }

    for(const auto& pv:m_vertices){
        ScanDataType::Ptr cloud(new ScanDataType());
        std::string pcd_file = QString(db_dir.path()+"/%1.pcd").arg(pv->id(), 10,10,QChar('0')).toStdString();
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"Reading "<<pcd_file<<".....\t";
#endif
        if(pcl::io::loadPCDFile<ScanDataType::PointType>(pcd_file,*cloud)<0){
            logger->logMessage((std::string("Failed to read ")+pcd_file).c_str());
            cloud.reset();
#ifdef GRAPH_SLAM_DEBUG
            std::cerr<<"failed"<<std::endl;
#endif
        }else{
#ifdef GRAPH_SLAM_DEBUG
            std::cerr<<"ok"<<std::endl;
#endif
        }
        m_scan_data.push_back(cloud);
    }


    if(m_g2o_graph->gaugeFreedom()){
        setVertexFixed(0,true);
        //            auto gauge = m_g2o_graph->findGauge();
        //            if(gauge == NULL){
        //                throw std::runtime_error("Cannot find a vertex to fix.");
        //            }
        //            gauge->setFixed(true);
        //    #ifdef GRAPH_SLAM_DEBUG
        //            std::cerr<<QString("Set vertex %1 fixed.").arg(gauge->id()).toStdString()<<std::endl;
        //    #endif
    }
}

bool GraphSLAM::saveAsG2O(const QString fname)
{
    std::ofstream file(fname.toStdString().c_str(), std::ios_base::trunc);
    bool ret = m_g2o_graph->save(file);
    return ret;
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

void GraphSLAM::removeEdge(const g2o::EdgeSE3 *e)
{
    g2o::EdgeSE3 * ee = const_cast<g2o::EdgeSE3*>(e);
    m_g2o_graph->removeEdge(ee);
    m_loop_edges.removeOne(ee);
    this->sendGraph();
}

void GraphSLAM::removeEdges(QList<const g2o::EdgeSE3 *> es)
{
    for(size_t i=0;i<es.size();i++){
        g2o::EdgeSE3 * ee = const_cast<g2o::EdgeSE3*>(es[i]);
        m_g2o_graph->removeEdge(const_cast<g2o::EdgeSE3*>(ee));
        m_loop_edges.removeOne(ee);
    }
    this->sendGraph();
}

void GraphSLAM::modifyEdges(const EdgeModifications &modifications)
{
    std::ostringstream oss;
    oss<<"Edges ";
    for(EdgeModifyAction::Ptr mod:modifications){
        g2o::EdgeSE3* edge = const_cast<g2o::EdgeSE3*>(mod->getEdge());
        oss<<"["<<edge->vertex(0)->id()<<"->"<<edge->vertex(1)->id()<<"] ";
        mod->action(const_cast<g2o::EdgeSE3*>(mod->getEdge()));
    }
    oss<<" modified.";
    logger->logMessage(oss.str().c_str());
}

void GraphSLAM::slot_sendGraphDisplay()
{
    if(receivers(SIGNAL(graphDisplayUpdated(GraphDisplayer::Ptr ))) < 1){
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"There is no connection(s)["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
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
    Q_EMIT graphDisplayUpdated(graph_disp);
}

void GraphSLAM::slot_sendGraphTable()
{
    if(receivers(SIGNAL(graphTableUpdated(GraphTableData*))) < 1){
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"There is no connection(s)["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }

    EdgeTableModel* loop_edges_table = new EdgeTableModel();
    EdgeTableModel * motion_edges_table = new EdgeTableModel();
    for(size_t i=0; i<m_loop_edges.size();i++){
        loop_edges_table->pushBack(m_loop_edges[i]);
    }
    for(size_t i=0; i<m_motion_edges.size();i++){
        motion_edges_table->pushBack(m_motion_edges[i]);
    }

    GraphTableData * graph_table = new GraphTableData();
    graph_table->setLoopEdges(loop_edges_table);
    graph_table->setMotionEdges(motion_edges_table);
    Q_EMIT graphTableUpdated(graph_table);
}

void GraphSLAM::sendGraph()
{
    slot_sendGraphDisplay();
    slot_sendGraphTable();
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

bool GraphSLAM::initialized() const
{
    return !m_vertices.empty();
}

g2o::EdgeSE3::Measurement GraphSLAM::getMotionConstraintBetweenVertices(int vi,  int vj)
{
    if(vi > vj){
        throw std::runtime_error("vi must be less or equal than vj");
    }
    g2o::EdgeSE3::Measurement constraint;
    constraint.setIdentity();
    for(int i=vi;i<vj;i++){
        constraint = constraint *  m_motion_edges[i]->measurement();
    }
    return constraint;
}

PointCloudDisplayer::Ptr GraphSLAM::getCompositedPointCloudDisplayer(QVector<int> vertices)
{
    PointCloudDisplayer::Ptr cloud(new PointCloudDisplayer());
    cloud->fields = {"x","y","z","intensity"};
    cloud->start = 0;

    GraphHelper::convertSE3ToqglviewerFrame(m_vertices[vertices[0]]->estimate(), cloud->frame);

    g2o::EdgeSE3::Measurement msr;
    qglviewer::Frame frame;
    msr.setIdentity();

    PointCloudDisplayer::CloudDataType point;
    point.resize(4);//x,y,z,intensity;

    for(const auto p:*(m_scan_data[vertices[0]])){
        point[0] = p.x;            point[1] = p.y;            point[2] = p.z;
        point[3] = p.intensity;
        cloud->pushBack(point);
    }
    qglviewer::Frame origin;
    origin.setPosition(0,0,0);
    origin.setOrientation(0,0,0,1);

    frame.setReferenceFrame(&origin);
    for(int i=1;i<vertices.size();i++){
        msr = msr * getMotionConstraintBetweenVertices(vertices[i-1], vertices[i]);
        GraphHelper::convertEdgeSE3ToqglviewerFrame(msr, frame);

        for(const auto p:*(m_scan_data[vertices[i]])){
            qglviewer::Vec pv(p.x, p.y, p.z);
            pv = frame.coordinatesOfIn(pv,&origin);
            point[0] = pv[0];            point[1] = pv[1];            point[2] = pv[2];
            point[3] = p.intensity;
            cloud->pushBack(point);
        }
    }
    return cloud;
}

void GraphSLAM::slot_selectVertices(const QList<int> &vertices_id, bool select_edges)
{
    DEBUG_FUNC_STAMP;
    if(receivers(SIGNAL(edgesSelected(QList<int>, QList<int>)))<1){
#ifdef GRAPH_SLAM_DEBUG
        std::cerr<<"There is no connection(s)["<<Q_FUNC_INFO<<"]"<<std::endl;
#endif
        return;
    }
    Q_EMIT verticesSelected(vertices_id);

    if(select_edges){
        //aggregate every edges stemming from selected vertices
        std::map<g2o::HyperGraph::Edge*, int> loop, motion;

        for(const auto& id:vertices_id){
            for(auto e:m_vertices[id]->edges()){
                if(abs(e->vertex(0)->id() - e->vertex(1)->id()) > 1){
                    //loop closing edge
                    if(loop.find(e) ==loop.end()){
                        loop[e] = 1;
                    }else{
                        loop[e]++;
                    }
                }
                else{
                    //motion edge
                    if(motion.find(e) ==motion.end()){
                        motion[e] = 1;
                    }else{
                        motion[e]++;
                    }
                }
            }
        }

        //For an edge being selected, both vertices should be selected.
        for(auto iter =loop.begin();iter !=loop.end();iter++){
            if(iter->second != 2){
                loop.erase(iter);
            }
        }
        for(auto iter =motion.begin();iter !=motion.end();iter++){
            if(iter->second != 2){
                motion.erase(iter);
            }
        }

        //find the index of selected edges in each edge vectors.

        QList<int> loop_ids, motion_ids;
        for(size_t i=0;i<m_loop_edges.size();i++){
            auto res = loop.find(m_loop_edges[i]);
            if(res!= loop.end()){
                loop_ids.push_back(i);
                loop.erase(res);
            }
        }
        for(size_t i=0;i<m_motion_edges.size();i++){
            auto res = motion.find(m_motion_edges[i]);
            if(res!= motion.end()){
                motion_ids.push_back(i);
                motion.erase(res);
            }
        }
        if(!loop.empty()){
            logger->logMessage(QString("There are remaining loop edges[%1]").arg(Q_FUNC_INFO).toStdString().c_str());
        }
        if(!motion.empty()){
            logger->logMessage(QString("There are remaining motion edges[%1]").arg(Q_FUNC_INFO).toStdString().c_str());
        }

        Q_EMIT edgesSelected(motion_ids,loop_ids);
    }
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


