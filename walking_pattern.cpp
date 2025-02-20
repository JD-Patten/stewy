#include "walking_pattern.h"

Pose WalkingPattern::getPose(float time) const {
    // Get values from all sin waves at current time
    float x = _x.getValue(time);
    float y = _y.getValue(time);
    float z = _z.getValue(time);
    float roll = _roll.getValue(time);
    float pitch = _pitch.getValue(time);
    float yaw = _yaw.getValue(time);

    // Return pose with current values
    return Pose{x, y, z, roll, pitch, yaw};
}

// Example walking patterns
namespace WalkingPatterns {

    // Basic walking pattern to test
    WalkingPattern z_motion(float amplitude, float period, float position) {
        sin_wave x(0, period, 0, 0);  // No x motion
        sin_wave y(0, period, 0, 0);  // No y motion
        sin_wave z(amplitude, period, position, 0);  // Up/down motion
        
        // No rotation
        sin_wave roll(0, period, 0, 0);
        sin_wave pitch(0, period, 0, 0);
        sin_wave yaw(0, period, 0, 0);

        return WalkingPattern(x, y, z, roll, pitch, yaw);
    }

} // namespace WalkingPatterns
