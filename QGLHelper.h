#ifndef QGLHELPER_H
#define QGLHELPER_H
#include <QGLViewer/frame.h>
#include <postypes.h>
#include <libicp/matrix.h>
namespace QGLHelper{

qglviewer::Frame getLocalTransformation(const qglviewer::Frame& from,const qglviewer::Frame& to);
qglviewer::Frame concatenateTransforms(const qglviewer::Frame& T1,const qglviewer::Frame& T2);

PosTypes::Pose3D toPosTypesPose3D(const qglviewer::Frame& frame);


qglviewer::Frame concatenateTransforms(const qglviewer::Frame &T1, const qglviewer::Frame &T2);
void qglFrameToLibicpMatrix(const qglviewer::Frame& frame, libicp::Matrix& R, libicp::Matrix& t);
void qglFrameToLibicpMatrix(const qglviewer::Frame& frame, libicp::Matrix& T);
void libicpMatrixToqglFrame( const libicp::Matrix& R, const libicp::Matrix& t, qglviewer::Frame& frame);

}

#endif // QGLHELPER_H
