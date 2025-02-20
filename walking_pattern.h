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

class WalkingPattern {
private:
    sin_wave _x;
    sin_wave _y;
    sin_wave _z;
    sin_wave _roll;
    sin_wave _pitch;
    sin_wave _yaw;
    
public:
    WalkingPattern(sin_wave x, sin_wave y, sin_wave z, 
                   sin_wave roll, sin_wave pitch, sin_wave yaw) :
        _x(x), _y(y), _z(z), _roll(roll), _pitch(pitch), _yaw(yaw) {}

    Pose getPose(float time) const;  
};

namespace WalkingPatterns {
    WalkingPattern z_motion(float amplitude, float period, float position);
} // namespace WalkingPatterns

#endif


