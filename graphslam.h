#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H
#include <QObject>
#include <vector>
#include "postypes.h"
#include "pointclouddisplayer.h"
#include "graphdisplayer.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <memory>
#include <map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "graphhelper.h"
#include "graphtablemodel.h"
namespace g2o{
class SparseOptimizer;
}

class GraphSLAM : public QObject
{
    Q_OBJECT
public:
    typedef std::shared_ptr<GraphSLAM> Ptr;
    typedef const std::shared_ptr<GraphSLAM> ConstPtr;
    typedef pcl::PointCloud<pcl::PointXYZI> ScanDataType;

    typedef g2o::VertexSE3 VertexType;
    typedef g2o::EdgeSE3 EdgeType;
    typedef EdgeType::InformationType InformationType;
public:
    GraphSLAM();

    size_t nVertices() const{
        return m_vertices.size();
    }

    size_t nEdges() const{
        return m_loop_edges.size() + m_motion_edges.size();
    }

    void init();
    void sendGraph();
    void sendPointCloud();

public Q_SLOTS:
    void reset();
    void resetLoopClosings();
    bool optimize(const int& n_iterations);
    bool loadFromG2OFile(const std::string &file_name);
    bool setVertexFixed(const int& id, const bool& fixed=true);

    /*
     * add a new vertex.
     */
    bool addVertex(const PosTypes::Pose3D& p, ScanDataType::Ptr cloud_ptr, const bool& incremental=true);

    /*
     * add a new edge.
     */
    bool addEdgeFromVertices(const int& vi, const int& vj, const g2o::EdgeSE3::InformationType& info_mat);

    bool addLoopClosing(const int& vi, const int& vj, const PosTypes::Pose3D& data ,const g2o::EdgeSE3::InformationType& info_mat);

    void removeEdge(g2o::EdgeSE3* e);
    void removeEdges(QList<g2o::EdgeSE3*> es);

Q_SIGNALS:
    void graphUpdated(GraphDisplayer::Ptr graph);
    void graphUpdated2(EdgeTableModel::Ptr graph_table_model);
    void pointCloudsUpdated(QVector<PointCloudDisplayer::Ptr> clouds);
    void showMessage(QString msg);

public Q_SLOTS:
    void slot_searchLoopClosings(QVector<int> idx_old,
                                 QVector<int> idx_new,
                                 PosTypes::Pose3D tr_new);
    void slot_startOptimize(int max_iterations);

protected:

    std::shared_ptr<g2o::SparseOptimizer> m_g2o_graph;
    QVector<g2o::VertexSE3*> m_vertices;
    QVector<ScanDataType::Ptr> m_scan_data;//indices must conincide with the indices of m_vertices.
    QVector<g2o::EdgeSE3*> m_motion_edges;
    QList<g2o::EdgeSE3*> m_loop_edges;
};

#endif // GRAPHSLAM_H
