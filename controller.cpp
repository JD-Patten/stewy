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
    // Initialize ESP32 PWM
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // attach servos with proper configuration
    for (int i = 0; i < 6; i++) {
        _servos[i].setPeriodHertz(50);    // Standard 50hz servo
        _servos[i].attach(servoPins[i], 600, 2400);  // Attach servo with min/max pulse widths
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

        // Apply offset
        float offsetAngle = checkAngle + _servoAngleOffsets[i];
        
        // Clamp angle between min and max
        float constrainedAngle = min(max(offsetAngle, _servoAngleMin), _servoAngleMax);
        
        // Convert back to publishing direction
        float publishAngle = (i % 2 == 0) ? -constrainedAngle : constrainedAngle;
        
        _servos[i].write(publishAngle + 90);
    }
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

void Controller::follow_trajectory() {

    // get the current pose on the trajectory
    Pose currentPose = _trajectory.currentPoseOnTrajectory();

    // update the current pose
    _currentPose = currentPose;
}

void Controller::set_trajectory(const Pose& currentPose, const Pose& goalPose) {
    // creates a trajectory that follows a straight line in the task space
    // with a 3rd order polynomial for the time scaling
    // reference Modern Robotics 9.2.1

    //float duration = 3.0;  // Fixed declaration
    _trajectory = Trajectory(currentPose, goalPose, _maxAcceleration, _maxAngularAcceleration);

    Serial.println("Trajectory created from: " + currentPose.toString());
    Serial.println("to: " + goalPose.toString());
    Serial.println("with duration: " + String(_trajectory._duration));
}

void Controller::update() {
    // control
    if (!_trajectory._isFinished) {
        follow_trajectory();


        // solve inverse kinematics
        IKResult result = _ikSolver.solveInverseKinematics(_currentPose);

        if (result.success) {
            publishToServos(result.angles);
        }
    }
    else {
        publishToServos(_ikSolver.solveInverseKinematics(_goalPose).angles);
    }
}

void Controller::setGoalPose(const Pose& goalPose) {
    // check if the goal pose is different from the current pose
    if (_goalPose == goalPose) {
        Serial.println("Goal pose is the same as the current pose");
        return;
    }

    // check if goal pose is valid
    IKResult result = _ikSolver.solveInverseKinematics(goalPose);
    if (!result.success) {
        Serial.println("Failed to solve inverse kinematics for goal pose: ");
        Serial.println(goalPose.toString());
        return;
    }

    _goalPose = goalPose; // update goal pose   
    Serial.println("Goal pose set to: "); 
    Serial.println(goalPose.toString());
    set_trajectory(_currentPose, _goalPose);
    
}

void Controller::setOffsets(vector<float> offsets) {
    _servoAngleOffsets = offsets;
    Serial.println("Servo angle offsets set to: ");
    for (int i = 0; i < 6; i++) {
        Serial.println(String(i) + ": " + String(_servoAngleOffsets[i]));
    }
}

void Controller::setAngleLimits(float min, float max) {
    _servoAngleMin = min;
    _servoAngleMax = max;
}

void Controller::setAccelerationLimits(float maxAcceleration, float maxAngularAcceleration) {
    _maxAcceleration = maxAcceleration;
    _maxAngularAcceleration = maxAngularAcceleration;

    Serial.println("Acceleration limits set to: " + String(maxAcceleration) + " mm/s^2 and " + String(maxAngularAcceleration) + " deg/s^2");
}

void Controller::setDampingFactor(float dampingFactor) {
    _dampingFactor = dampingFactor;
}

Trajectory::Trajectory(Pose startPose, Pose goalPose, float duration)
    : _startPose(startPose)
    , _goalPose(goalPose)
    , _duration(duration)
    , _startTime(millis())
    , _isFinished(false)
{
}

Trajectory::Trajectory(Pose startPose, Pose goalPose, float maxAcceleration, float maxAngularAcceleration)
    : _startPose(startPose)
    , _goalPose(goalPose)
    , _startTime(millis())
    , _isFinished(false)
{
    _duration = minimizeDuration(maxAcceleration, maxAngularAcceleration);
}

Pose Trajectory::currentPoseOnTrajectory() {
    // use a 3rd order polynomial for time scaling
    // reference Modern Robotics 9.2.1

    // calculate the time along the trajectory
    unsigned long currentTime = millis();
    float t = (currentTime - _startTime) / 1000.0f;

    if (t > _duration) {
        _isFinished = true;
        Serial.println("Trajectory finished");
    }

    // calculate the path parameter with 3rd order polynomial time scaling
    float s = (3 * t*t / (_duration * _duration)) - 
              (2 * t*t*t / (_duration * _duration * _duration));


    // Clamp s between 0 and 1
    s = min(max(s, 0.0f), 1.0f);

    return pointAlongPath(s);
}

float Trajectory::minimizeDuration(float maxAcceleration, float maxAngularAcceleration) {
    // minimize the duration of the trajectory
    // while keeping the max acceleration within the limit
    // reference Modern Robotics 9.2.1 p.330


    float translationalDistance = sqrt(pow(_goalPose.x - _startPose.x, 2) + 
                                      pow(_goalPose.y - _startPose.y, 2) + 
                                      pow(_goalPose.z - _startPose.z, 2));

    float angularDistance = sqrt(pow(_goalPose.roll - _startPose.roll, 2) + 
                                  pow(_goalPose.pitch - _startPose.pitch, 2) + 
                                  pow(_goalPose.yaw - _startPose.yaw, 2));

    float translationalDuration = sqrt(6 * translationalDistance / maxAcceleration);
    float angularDuration = sqrt(6 * angularDistance / maxAngularAcceleration);

    return max(translationalDuration, angularDuration);
}

Pose Trajectory::pointAlongPath(float s) {
    // s is the distance along the path
    // 0 <= s <= 1

    // Clamp s between 0 and 1
    s = min(max(s, 0.0f), 1.0f);

    // get the position along the path
    return _startPose + s * (_goalPose - _startPose);
}