#ifndef WALKING_PATTERN_H
#define WALKING_PATTERN_H

#include "inverse_kinematics.h"
#include <stdexcept>

struct sin_wave {
    float _amplitude;
    float _period;
    float _position_shift;
    float _time_shift;

    sin_wave(float amplitude, float period, float position_shift, float time_shift = 0.0f) {
        if (period == 0.0f) {
            throw std::invalid_argument("Period cannot be zero");
        }
        _amplitude = amplitude;
        _period = period;
        _position_shift = position_shift;
        _time_shift = time_shift;
    }

    float getValue(float time) const {
        return _amplitude * sin(2 * M_PI * (time + _time_shift) / _period) + _position_shift;
    }
};

class WalkingPattern1{
    private:
        sin_wave _x;
        sin_wave _y;
        sin_wave _z;
        sin_wave _roll;
        sin_wave _pitch;
        sin_wave _yaw;
        IKSolver _ikSolver;
        
        Pose getPose(float time) const;
    public:
        WalkingPattern1();
        vector<float> getAngles(float time, String direction, float speed) const;
};

#endif


