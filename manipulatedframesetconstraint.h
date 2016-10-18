#ifndef MANIPULATEDFRAMESETCONSTRAINT_H
#define MANIPULATEDFRAMESETCONSTRAINT_H
#include <QGLViewer/constraint.h>
#include <QGLViewer/frame.h>
#include <QSet>
class ManipulatedFrameSet{
public:
    void clear(){
            frames.clear();
    }

    void add(qglviewer::Frame* frame){
        frames.insert(frame);
    }

protected:
    typedef QSet<qglviewer::Frame*> FrameContainer;
    FrameContainer frames;
};

template <class ConstraintType>
class ManipulatedFrameSetConstraint: public ConstraintType,public ManipulatedFrameSet
{
public:
    virtual void constrainTranslation(qglviewer::Vec& tr, qglviewer::Frame* const frame);
    virtual void constrainRotation(qglviewer::Quaternion& quat, qglviewer::Frame* const frame);
};

typedef ManipulatedFrameSetConstraint<qglviewer::LocalConstraint> LocalManipulatedFrameSetConstraint;
typedef ManipulatedFrameSetConstraint<qglviewer::WorldConstraint> WorldManipulatedFrameSetConstraint;


///////////////////////Template implemenation//////////////////////////////////
template <class ConstraintType>
void ManipulatedFrameSetConstraint<ConstraintType>::constrainTranslation(qglviewer::Vec &tr, qglviewer::Frame * const frame){
    ConstraintType::constrainTranslation(tr, frame);
    for(FrameContainer::iterator it=frames.begin();it!=frames.end();it++){
        (*it)->translate(tr);
    }
}
template <class ConstraintType>
void ManipulatedFrameSetConstraint<ConstraintType>::constrainRotation(qglviewer::Quaternion &quat, qglviewer::Frame * const frame){
    ConstraintType::constrainRotation(quat, frame);
    const qglviewer::Vec world = frame->inverseTransformOf(quat.axis());
    const qglviewer::Vec pos = frame->position();
    const float angle = quat.angle();

    for(FrameContainer::iterator it=frames.begin();it!=frames.end();it++){
        qglviewer::Quaternion q((*it)->transformOf(world), angle);
        (*it)->rotate(q);

        qglviewer::Quaternion q_world(world, angle);
        (*it)->setPosition(pos + q_world.rotate((*it)->position() - pos));
    }
}

#endif // MANIPULATEDFRAMESETCONSTRAINT_H
