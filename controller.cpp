#include "controller.h"

Controller::Controller(int servoPins[6], IKSolver ikSolver, float kp, float ki, float kd, float maxAcceleration, float maxAngularAcceleration)
    : _kp(kp)
    , _ki(ki)
    , _kd(kd)
    , _ikSolver(ikSolver)
    , _servoAngleOffsets(6, 0.0)
    , _servoAngleMin(-80.0)             //degrees
    , _servoAngleMax(80.0)              //degrees
    , _maxAcceleration(10.0)            //mm/s^2
    , _maxAngularAcceleration(10.0)     //degrees/s^2
    , _plotCounter(0)
{
    // attach servos
    for (int i = 0; i < 6; i++) {
        _servos[i].attach(servoPins[i]);
    }
}
        
void Controller::begin(const Pose& initialPose) {
    _goalPose = initialPose;

    // solve inverse kinematics
    IKResult result = _ikSolver.solveInverseKinematics(initialPose);

    if (result.success) {
        // initialize variables
        _goalPose = initialPose;
        _currentPose = initialPose;
        _previousErrors = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        _integrals = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        _previousVelocity = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        _previousTime = millis();

        // publish to servos    
        publishToServos(result.angles);

    } else {
        Serial.println("Failed to solve inverse kinematics");
    }
}

void Controller::publishToServos(const vector<float>& angles) {
    for (int i = 0; i < 6; i++) {
        // For odd servos, check limits on positive value but publish negative
        float checkAngle = (i % 2 == 0) ? -angles[i] : angles[i];
        
        // Clamp angle between min and max
        float constrainedAngle = min(max(checkAngle, _servoAngleMin), _servoAngleMax);
        
        // Convert back to publishing direction
        float publishAngle = (i % 2 == 0) ? -constrainedAngle : constrainedAngle;
        
        _servos[i].write(publishAngle + 90);
    }
    delay(10);
}

void Controller::calculatePIDControl() {
    // calculate error
    Pose error = _goalPose - _currentPose;
    
    // calculate integral
    _integrals += error;
    
    // calculate derivative
    Pose derivative = error - _previousErrors;
    _previousErrors = error;
    
    // calculate output
    Pose output = error * _kp + _integrals * _ki + derivative * _kd;
    
    // update current pose
    _currentPose += output;
}

void Controller::capAcceleration() {
    unsigned long currentTime = millis();
    float dt = (currentTime - _previousTime) / 1000.0;  // Convert to seconds
    _previousTime = currentTime;

    //temporary just move to goal pose
    _currentPose = 0.1 * (_goalPose - _currentPose) + _currentPose;

}

void Controller::update() {
    // control
    capAcceleration();

    // solve inverse kinematics
    IKResult result = _ikSolver.solveInverseKinematics(_currentPose);

    if (result.success) {
        publishToServos(result.angles);
    }
}

void Controller::setGoalPose(const Pose& goalPose) {
    // check if goal pose is valid
    IKResult result = _ikSolver.solveInverseKinematics(goalPose);
    if (result.success){
        _goalPose = goalPose; // update goal pose   
        Serial.println("Goal pose set to: ");
        Serial.println(goalPose.toString());
    }
}

void Controller::setOffsets(vector<float> offsets) {
    _servoAngleOffsets = offsets;
}

void Controller::setAngleLimits(float min, float max) {
    _servoAngleMin = min;
    _servoAngleMax = max;
}

void Controller::setAccelerationLimits(float maxAcceleration, float maxAngularAcceleration) {
    _maxAcceleration = maxAcceleration;
    _maxAngularAcceleration = maxAngularAcceleration;
}

void Controller::setDampingFactor(float dampingFactor) {
    _dampingFactor = dampingFactor;
}