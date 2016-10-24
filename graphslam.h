#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H
#include <QObject>
#include <vector>
#include "postypes.h"
#include "pointclouddisplayer.h"
#include "graphdisplayer.h"
#include <g2o/types/slam2d/types_slam2d.h>
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

    typedef g2o::VertexSE2 VertexType;
    typedef g2o::EdgeSE2 EdgeType;
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
    PointCloudDisplayer::Ptr getCompositedPointCloudDisplayer(QVector<int> vertices, bool using_edges);
    void getPointCloudDisplayers(const QVector<int>& vertices, QVector<PointCloudDisplayer::Ptr>& cloud_out);
    void removeLastLoopEdge();
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
    bool addEdgeFromVertices(const int& vi, const int& vj, const EdgeType::InformationType& info_mat);
    bool addLoopClosing(const int& vi, const int& vj, const PosTypes::Pose2D& data ,const EdgeType::InformationType& info_mat);

    //Removing edge
    void removeLoopEdges(const QVector<int>& indices);
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
    EdgeType::Measurement getMotionConstraintBetweenVertices(int vi, int vj);
protected:
    std::shared_ptr<g2o::SparseOptimizer> m_g2o_graph;
    QVector<VertexType*> m_vertices;
    QVector<ScanDataType::Ptr> m_scan_data;//indices must conincide with the indices of m_vertices.
    QVector<EdgeType*> m_motion_edges;
    QList<EdgeType*> m_loop_edges;
    friend class MainWindow;
};

#endif // GRAPHSLAM_H
