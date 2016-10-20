#ifndef POINTCLOUDDISPLAYER_H
#define POINTCLOUDDISPLAYER_H
#include <memory>
#include <vector>
#include <QGLViewer/manipulatedCameraFrame.h>

class PointCloudDisplayer
{
public:
    typedef std::shared_ptr<PointCloudDisplayer> Ptr;
    typedef const std::shared_ptr<const PointCloudDisplayer> ConstPtr;
    typedef std::vector<GLfloat> CloudDataType;
public:
    PointCloudDisplayer();
    void setSelectable(bool selectable=true){
        this->selectable=selectable;
    }
    bool isSelectable()const {return selectable;}
    void drawPointCloud(const bool& selected,
                        const GLfloat& pointSize,
                        const GLdouble& alpha);
    void drawFrame(const bool& selected);


    void copyXYZTo(std::vector<double>& output);
    void pushBack(const CloudDataType& p);
    CloudDataType getPoint(size_t i) const;
    size_t nPoints() const;

//    size_t n_points;
    qglviewer::Frame frame;
    CloudDataType data;
    int start;//index of the beginning of xyz fields.
    std::vector<std::string> fields;//how many fields does a point have?
    bool selectable;
protected:

};


#endif // POINTCLOUDDISPLAYER_H
