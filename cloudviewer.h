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

class CloudViewer: public QGLViewer
{
    Q_OBJECT
public:
    typedef  std::vector<PointCloudDisplayer::Ptr> PointCloudDisplayerVector;

    CloudViewer(QWidget* parent):QGLViewer(parent){

    }

public Q_SLOTS:
    void setGraphDisplayer(GraphDisplayer::Ptr graph){
        logger->logMessage("setGraphDisplayer");
        this->graph = graph;
    }
    void setPointCloudDisplayers(QVector<PointCloudDisplayer::Ptr> clouds){
        logger->logMessage("setPointCloudDisplayers");
        this->clouds = clouds;
    }

Q_SIGNALS:
    /*
     *  perform LoopClosingSearch after transforming new vertices(idxNew).
     *  trNew is used for giving initial guess of scan matching algorithms.
     */
    void searchLoopClosings(std::vector<int> idxOld,
                            std::vector<int> idxNew,
                            qglviewer::Frame trNew);

protected:
    virtual void init();
    virtual void draw();
    virtual void drawWithNames();
    virtual void endSelection(const QPoint &point);

    // Mouse events functions
    virtual void mousePressEvent(QMouseEvent *e);
    virtual void mouseMoveEvent(QMouseEvent *e);
    virtual void mouseReleaseEvent(QMouseEvent *e);

    void startManipulation();
    void drawSelectionRectangle() const;
    void addIDToSelections(int id);
    void removeIDFromSelections(int id);
private:
    QRect rectangle;
    enum SelectionMode{NONE, ADD, REMOVE};
    SelectionMode selection_mode;

    QList<int> selections;
    QVector<PointCloudDisplayer::Ptr> clouds;
    GraphDisplayer::Ptr graph;

};

#endif // CLOUDVIEWER_H
