#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <vector>
#include "inverse_kinematics.h"


class Trajectory {
private:
    Pose _startPose;
    Pose _goalPose;
    float _duration;
    unsigned long _startTime;

    float minimizeDuration(float maxAcceleration, float maxAngularAcceleration);

public:
    bool _isFinished;
    Pose pointAlongPath(float s);
    Trajectory() {}
    Trajectory(Pose startPose, Pose goalPose, float duration);
    Trajectory(Pose startPose, Pose goalPose, float maxAcceleration, float maxAngularAcceleration);
    Pose currentPoseOnTrajectory();
};

class Controller {
private:
    // Hardware components
    Servo _servos[6];
    vector<float> _servoAngleOffsets;
    float _servoAngleMin;
    float _servoAngleMax;
    float _maxAcceleration;
    float _maxAngularAcceleration;

    // controller parameters
    float _kp, _ki, _kd;
    Pose _integrals;
    Pose _previousErrors;
    Pose _previousVelocity;
    float _dampingFactor;
    unsigned long _previousTime;
    int _plotCounter;

    // Inverse Kinematics solver
    IKSolver _ikSolver;

    // top plate poses
    Pose _currentPose; // x, y, z, roll, pitch, yaw
    Pose _goalPose;    // x, y, z, roll, pitch, yaw

    Trajectory _trajectory;

    void calculatePIDControl();
    void follow_trajectory();
    void publishToServos(const vector<float>& angles);
    void set_trajectory(const Pose& currentPose, const Pose& goalPose);

public:
    // Constructor
    Controller(int servoPins[6], IKSolver ikSolver, float kp, float ki, float kd, float maxAcceleration, float maxAngularAcceleration);
    void begin(const Pose& initialPose);
    void setGoalPose(const Pose& goalPose);
    void setAccelerationLimits(float maxAcceleration, float maxAngularAcceleration);
    void setDampingFactor(float dampingFactor);
    void setOffsets(vector<float> offsets);
    void setAngleLimits(float min, float max);
    void update();
};

#endif 
