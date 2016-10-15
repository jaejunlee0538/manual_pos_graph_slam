#include "manipulatedframesetconstraint.h"

void ManipulatedFrameSetConstraint::clear(){
    frames.clear();
}

void ManipulatedFrameSetConstraint::add(qglviewer::Frame *frame){
    frames.insert(frame);
}

void ManipulatedFrameSetConstraint::constrainTranslation(qglviewer::Vec &tr, qglviewer::Frame * const frame){
    std::cerr<<"constrainTranslation : "<<tr<<std::endl;
    for(FrameContainer::iterator it=frames.begin();it!=frames.end();it++){
        (*it)->translate(tr);
    }
}

void ManipulatedFrameSetConstraint::constraintRotation(qglviewer::Quaternion &quat, qglviewer::Frame * const frame){

        std::cerr<<"constraintRotation : "<<quat<<std::endl;
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
