#ifndef WALKING_PATTERN_H
#define WALKING_PATTERN_H

#include "inverse_kinematics.h"


class WalkingPattern {
private:
    Pose _poses[];
    float _dt;
public:
    WalkingPattern(vector<Pose> poses, float dt);
    Pose getPose(float time);
};


#endif


