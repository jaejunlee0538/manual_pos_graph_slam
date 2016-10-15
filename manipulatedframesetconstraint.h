#ifndef MANIPULATEDFRAMESETCONSTRAINT_H
#define MANIPULATEDFRAMESETCONSTRAINT_H
#include <QGLViewer/constraint.h>
#include <QGLViewer/frame.h>
#include <QSet>
class ManipulatedFrameSetConstraint: public qglviewer::Constraint
{
public:
    void clear();
    void add(qglviewer::Frame* frame);

    virtual void constrainTranslation(qglviewer::Vec& tr, qglviewer::Frame* const frame);

    virtual void constraintRotation(qglviewer::Quaternion& quat, qglviewer::Frame* const frame);
private:
    typedef QSet<qglviewer::Frame*> FrameContainer;
    FrameContainer frames;
};

#endif // MANIPULATEDFRAMESETCONSTRAINT_H
