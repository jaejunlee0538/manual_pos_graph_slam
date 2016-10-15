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

    void drawPointCloud(const bool& selected,
                        const double& pointSize,
                        const double& alpha);
    void drawFrame(const bool& selected);

    qglviewer::Frame frame;
    CloudDataType data;
    int start;//index of the beginning of xyz fields.
    std::vector<std::string> fields;//how many fields does a point have?
};

#endif // POINTCLOUDDISPLAYER_H
