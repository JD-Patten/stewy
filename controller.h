#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <vector>
#include "inverse_kinematics.h"
#include "walking_pattern.h"

class Trajectory {
private:
    Pose _startPose;
    Pose _goalPose;
    unsigned long _startTime;
    float minimizeDuration(float maxAcceleration, float maxAngularAcceleration);

public:
    float _duration;
    bool _isFinished;
    Trajectory() {}
    Trajectory(Pose startPose, Pose goalPose, float maxAcceleration, float maxAngularAcceleration);
    Pose currentPoseOnTrajectory();
    Pose pointAlongPath(float s);
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

    WalkingPattern _walkingPattern;
    

    // Inverse Kinematics solver
    IKSolver _ikSolver;

    // top plate poses
    Pose _currentPose;
    Pose _goalPose;

    Trajectory _trajectory;

    void calculatePIDControl();
    void follow_trajectory();
    void publishToServos(const vector<float>& angles);
    void set_trajectory(const Pose& currentPose, const Pose& goalPose);
    void walk();

public:
    bool _isWalking;
    String _walkingDirection;
    float _speedMultiplier;

    Controller(int servoPins[6], IKSolver ikSolver, float maxAcceleration, float maxAngularAcceleration);
    void begin(const Pose& initialPose);
    void setGoalPose(const Pose& goalPose);
    void startWalking(String direction, float speedMultiplier);
    void setAccelerationLimits(float maxAcceleration, float maxAngularAcceleration);
    void setDampingFactor(float dampingFactor);
    void setOffsets(vector<float> offsets);
    void setAngleLimits(float min, float max);
    void update();
    vector<float> getServoAngles();
};

#endif 
