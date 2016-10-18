#ifndef CLOUDVIEWER_H
#define CLOUDVIEWER_H
#include <QGLViewer/qglviewer.h>
#include <vector>
#include <memory>
#include <QMessageBox>
#include <QGLViewer/manipulatedFrame.h>
#include "pointclouddisplayer.h"
#include "graphdisplayer.h"
#include <mutex>
#include "Global.h"
#include <QList>
#include <g2o/types/slam3d/edge_se3.h>
#include <QSet>
#include "selectioninfo.h"
#include "textdrawhelper.h"
class CloudViewer: public QGLViewer
{
    Q_OBJECT
public:
    typedef  QVector<PointCloudDisplayer::Ptr> PointCloudDisplayerVector;

    CloudViewer(QWidget* parent);
    const QList<int>& getSelections() const{
        return selections;
    }
public Q_SLOTS:
    void setGraphDisplayer(GraphDisplayer::Ptr graph);
    void setPointCloudDisplayers(QVector<PointCloudDisplayer::Ptr> clouds);
    void slot_manualLoopClosing();
    void slot_setPointAlpha(const double& alpha_point_cloud);
    void slot_setPointSize(const double& point_size);

Q_SIGNALS:
    /*
     *  perform LoopClosingSearch after transforming new vertices(idxNew).
     *  trNew is used for giving initial guess of scan matching algorithms.
     */
    void searchLoopClosings(std::vector<int> idxOld,
                            std::vector<int> idxNew,
                            qglviewer::Frame trNew);
    void loopClosingAdded(const int& vi, const int& vj, const PosTypes::Pose3D& data ,const g2o::EdgeSE3::InformationType& info_mat);
    void selectionChanged(GraphSelectionInfo selected);
protected:
    virtual void init();
    virtual void draw();
    virtual void drawWithNames();
    virtual void endSelection(const QPoint &point);

    // Mouse events functions
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);
    //Keyboard event functions
    virtual void keyPressEvent(QKeyEvent* key);

    void startManipulation();
    void drawSelectionRectangle() const;
    void addIDToSelections(int id);
    void removeIDFromSelections(int id);
    void drawText();

    //Frame constraints functions.
    void switchTranslationConstraintType();
    void switchRotationConstraintType();
    void switchConstraint();
    void switchConstraintDirection(bool rotation);

private:
    QRect rectangle;
    enum SelectionMode{NONE, ADD, REMOVE};
    SelectionMode selection_mode;

    QList<int> selections;
    QVector<PointCloudDisplayer::Ptr> clouds;
    GraphDisplayer::Ptr graph;
    GLdouble alpha_points;
    GLfloat size_points;

    qglviewer::AxisPlaneConstraint* constraints[2];
    TextDrawHelper constraint_info_text;
    int active_constraint;
    int translation_dir, rotation_dir;
};

#endif // CLOUDVIEWER_H
