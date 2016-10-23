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
#include <unordered_map>
#include <QDir>
#include "edgemanipulator.h"
#include <g2o/core/robust_kernel.h>
namespace g2o{
class SparseOptimizer;
}
class MainWindow;
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
    bool initialized()const;
    PointCloudDisplayer::Ptr getCompositedPointCloudDisplayer(QVector<int> vertices);
    void getPointCloudDisplayers(const QVector<int>& vertices, QVector<PointCloudDisplayer::Ptr>& cloud_out);
public Q_SLOTS:
    void reset();
    void resetLoopClosings();
    bool optimize(const int& n_iterations);
    void loadFromG2ODB(const QDir &db_dir);
    bool saveAsG2O(const QString fname);
    bool setVertexFixed(const int& id, const bool& fixed=true);

    //Adding vertex
    bool addVertex(const PosTypes::Pose3D& p, ScanDataType::Ptr cloud_ptr, const bool& incremental=true);

    //Adding Edge
    bool addEdgeFromVertices(const int& vi, const int& vj, const g2o::EdgeSE3::InformationType& info_mat);
    bool addLoopClosing(const int& vi, const int& vj, const PosTypes::Pose3D& data ,const g2o::EdgeSE3::InformationType& info_mat);

    //Removing edge
    void removeEdge(const g2o::EdgeSE3* e);
    void removeEdges(QList<const g2o::EdgeSE3*> es);
    void modifyEdges(const EdgeModifications& modifications);
Q_SIGNALS:
    void graphDisplayUpdated(GraphDisplayer::Ptr graph);
    void graphTableUpdated(GraphTableData* graph_table_model);
    void pointCloudsUpdated(QVector<PointCloudDisplayer::Ptr> clouds);
    void showMessage(QString msg);
    void verticesSelected(const QList<int>& vertices_id);
    void edgesSelected(const QList<int>& motion_edges, const QList<int>& loop_edges);

public Q_SLOTS:
    void slot_sendGraphTable();
    void slot_sendGraphDisplay();
    void slot_selectVertices(const QList<int>& vertices_id, bool select_edges);

protected:
    void loadPCDFiles();
    void initG2OGraph();
    g2o::EdgeSE3::Measurement getMotionConstraintBetweenVertices(int vi, int vj);
protected:
    std::shared_ptr<g2o::SparseOptimizer> m_g2o_graph;
    QVector<g2o::VertexSE3*> m_vertices;
    QVector<ScanDataType::Ptr> m_scan_data;//indices must conincide with the indices of m_vertices.
    QVector<g2o::EdgeSE3*> m_motion_edges;
    QList<g2o::EdgeSE3*> m_loop_edges;
    friend class MainWindow;
};

#endif // GRAPHSLAM_H
