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
    
    inline WalkingPattern wp1() {
        return WalkingPattern(
            sin_wave(10.952f, 1.0f, 11.508f, 0.54f),    // x
            sin_wave(10.476f, 2.0f, -9.921f, 0.365f),   // y
            sin_wave(25.238f, 1.0f, 89.206f, 0.476f),   // z
            sin_wave(19.048f, 1.0f, -4.048f, 0.603f),   // roll
            sin_wave(2.54f, 1.0f, 0.714f, 0.286f),      // pitch
            sin_wave(3.81f, 1.0f, -13.095f, 0.952f)     // yaw
        );
    }
} // namespace WalkingPatterns

#endif


