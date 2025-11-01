#include "controller.h"

Controller::Controller(int servoPins[6], IKSolver ikSolver, float maxAcceleration, float maxAngularAcceleration)
    : _ikSolver(ikSolver)
    , _servoAngleOffsets(6, 0.0)
    , _servoAngleMin(-80.0)             //degrees
    , _servoAngleMax(80.0)              //degrees
    , _maxAcceleration(maxAcceleration)
    , _maxAngularAcceleration(maxAngularAcceleration)
    , _isWalking(false)
    , _walkingDirection("no direction")
    , _walkingPattern(WalkingPattern1())
    , _speedMultiplier(1.0)
    , _lastCommandedAngles(6, 0.0)
    , _currentMode(STANDARD)
    , _inVelocityMode(false)
    , _lastSensorTime(millis())
{
    // Initialize ESP32 PWM
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // attach servos with proper configuration
    for (int i = 0; i < 6; i++) {
        _servos[i].setPeriodHertz(50);    // Standard 50hz servo
        _servos[i].attach(servoPins[i], 500, 2500);  // Attach servo with min/max pulse widths
    }

    _integratedOffsets = Pose();
}
        
void Controller::begin(const Pose& initialPose) {
    _goalPose = initialPose;

    // solve inverse kinematics
    IKResult result = _ikSolver.solveInverseKinematics(initialPose);

    if (result.success) {
        _goalPose = initialPose;
        _currentPose = initialPose;
        publishToServos(result.angles);
    } else {
        Serial.println("Failed to solve inverse kinematics");
    }
}

void Controller::publishToServos(const vector<float>& angles) {
    // Debug print for angles being sent to servos
    
    for (int i = 0; i < 6; i++) {
        // For odd servos, check limits on positive value but publish negative
        float checkAngle = (i % 2 == 0) ? -angles[i] : angles[i];

        // Apply offset
        float offsetAngle = checkAngle + _servoAngleOffsets[i];
        
        // Clamp angle between min and max
        float constrainedAngle = min(max(offsetAngle, _servoAngleMin), _servoAngleMax);
        
        // Convert back to publishing direction
        float publishAngle = (i % 2 == 0) ? -constrainedAngle : constrainedAngle;

        // Store the commanded angle (without the 90 degree offset)
        _lastCommandedAngles[i] = angles[i];
        
        _servos[i].write(publishAngle + 90);
    }
}

void Controller::updateSensorState(float raw_distance, float raw_joyX, float raw_joyY, bool raw_joyClick) {
    // update internal sensor states
    _prevJoyClick = _joyClick;
    _joyClick = raw_joyClick;
    _distance = raw_distance;

    // recenter joystick values and apply deadzone
    float xCenter = 2160;
    float yCenter = 2200;
    float deadzone = 25;

    _joyX = raw_joyX - xCenter;
    if (abs(_joyX) < deadzone) _joyX = 0;
    _joyY = raw_joyY - yCenter;
    if (abs(_joyY) < deadzone) _joyY = 0;

    // Shared dt computation for sensor integrations
    unsigned long now = millis();
    float dt = (now - _lastSensorTime) / 1000.0f;
    _lastSensorTime = now;
    if (dt > 0.1f) dt = 0.1f;

    float magnitude = sqrt(_joyX * _joyX + _joyY * _joyY);
    if (magnitude > 2000) magnitude = 2000;


    // unit vectors for seelcting a direction based on joystick position
    // left and right are back left and back right faces of the robot
    vector<pair<String, pair<float, float>>> directions = {
        {"forward", {0 , 1.0}},
        {"left", {0.866025, -0.5}},
        {"right", {-0.866025, -0.5}},
        {"backward", {0 , -1.0}},
        {"forward left", {0.866025, 0.5}},
        {"forward right", {-0.866025, 0.5}}
    };

    // if the joystick was just clicked change the "mode" of the robot
    if (_joyClick && !_prevJoyClick) {
        _currentMode = static_cast<ControlMode>((_currentMode + 1) % 5);
        Serial.println("Control mode: " + getModeString());
        _isWalking = false;  // stop walking when mode changes
    }

    // Set velocity mode based on current mode
    if (_currentMode == JOYSTICK_X || _currentMode == JOYSTICK_Y || _currentMode == JOYSTICK_Z) {
        _inVelocityMode = true;
    } else {
        _inVelocityMode = false;
    }

    float largestDot = 0.0;
    bool flipDirection = false;
    String newWalkingDirection = "none";
    // use the dot product to determine which direction to walk in
    for (auto& dir : directions) {
        float dot = (_joyX * dir.second.first + _joyY * dir.second.second);

        if (dot > largestDot) {
            if (dir.first == "backward") {
                newWalkingDirection = "forward";
                flipDirection = true;
            } else if (dir.first == "forward left") {
                newWalkingDirection = "right";
                flipDirection = true;
            } else if (dir.first == "forward right") {
                newWalkingDirection = "left";
                flipDirection = true;
            } else {
                newWalkingDirection = dir.first;
            }
            largestDot = dot;
            }
        }


    // if in walking mode, send walk command based on joystick position
    if (_currentMode == JOYSTICK_WALK) {

        // now set walking
        if (magnitude > 0) {
            float velocity = largestDot * 1.0 / 2000.0;
            if (flipDirection == true) velocity *= -1.0;

            if (_isWalking && _walkingDirection == newWalkingDirection) {
                // already walking in this direction just update the speed
                _walkingDirection = newWalkingDirection;
                _speedMultiplier = velocity;
            } else{
                _walkingDirection = newWalkingDirection;
                _speedMultiplier = velocity;
                startWalking(_walkingDirection, _speedMultiplier);
                Serial.println("Started walking " + String(_walkingDirection) + " with speed multiplier: " + String(_speedMultiplier));
            }
        } else {
            _isWalking = false;
        }
    }


    // Print-only basic integration test for JOYSTICK_X/Y/Z modes
    if (_inVelocityMode && _trajectory._isFinished) {
        if (dt > 0.0f) {
            // Tuning constants
            const float joy_max = 2000.0f;
            const float max_trans_vel = 70.0f;  // mm/s
            const float max_rot_vel = 50.0f;    // deg/s

            float x_norm = _joyX / joy_max;
            float y_norm = _joyY / joy_max;
            float trans_delta = 0.0f;
            float rot_delta = 0.0f;

            // Mode-specific DOFs
            int trans_dof = -1, rot_dof = -1;
            if (_currentMode == JOYSTICK_X) {
                trans_delta = -y_norm * max_trans_vel * dt;
                rot_delta = x_norm * max_rot_vel * dt;
                trans_dof = 0;  // x translation
                rot_dof = 3;    // roll rotation
            } else if (_currentMode == JOYSTICK_Y) {
                trans_delta = -x_norm * max_trans_vel * dt;
                rot_delta = -y_norm * max_rot_vel * dt;
                trans_dof = 1;  // y
                rot_dof = 4;    // pitch
            } else if (_currentMode == JOYSTICK_Z) {
                trans_delta = y_norm * max_trans_vel * dt;
                rot_delta = x_norm * max_rot_vel * dt;
                trans_dof = 2;  // z
                rot_dof = 5;    // yaw
            }

            Pose unchecked_offsets(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            unchecked_offsets[trans_dof] = trans_delta;
            unchecked_offsets[rot_dof] = rot_delta;

            // Test tentative integration
            Pose tentative = _integratedOffsets + unchecked_offsets;
            Pose testPose = _currentPose + tentative;

            IKResult testResult = _ikSolver.solveInverseKinematics(testPose);
            if (testResult.success) {
                // Commit accumulation if valid
                _integratedOffsets += unchecked_offsets;
                //Serial.println("Integrated offsets: " + _integratedOffsets.toString());
            } else {
                Serial.println("Skipped invalid offsets: " + unchecked_offsets.toString());
            }
        }
    }

    // Collision avoidance offset (persistent, velocity-based)
    float target_offset = 0.0f;
    if (_distance < 5.0f) {
        target_offset = 150.0f;
    }
    if (dt > 0.0f) {
        float avoidance_velocity = 150.0f;  // mm/s, tunable
        float delta = 0.0f;
        if (target_offset > _collisionOffsetMagnitude) {
            delta = avoidance_velocity * dt;
        } else if (target_offset < _collisionOffsetMagnitude) {
            delta = -avoidance_velocity * dt;
        }
        _collisionOffsetMagnitude += delta;
        _collisionOffsetMagnitude = max(0.0f, min(150.0f, _collisionOffsetMagnitude));
    }

    // Set collision pose offset
    float pitch = (20 + _currentPose.pitch) * 3.14159 / 180.0;
    float yaw = _currentPose.yaw * 3.14159 / 180.0;
    _collisionOffsets = Pose(_collisionOffsetMagnitude * cos(pitch) * cos(yaw) , _collisionOffsetMagnitude * cos(pitch) * sin(yaw), -1 * _collisionOffsetMagnitude * sin(pitch), 0, 0, 0);
}

void Controller::walk() {

    // get the angles for the walking pattern   
    vector<float> angles = _walkingPattern.getAngles(_walkingDirection, _speedMultiplier);

    // fade in to transition into the walking pattern
    float speed = _speedMultiplier;
    if (speed < 0.0) speed *= -1.0;

    float fadeInDuration = 0.2 / speed;
    float t = millis() / 1000.0f;
    float timeSinceWalkingStarted = t - _walkingStartTime;
    float fadeInPercent = min(timeSinceWalkingStarted / fadeInDuration, 1.0f);
    for (int i = 0; i < 6; i++) {
        angles[i] = angles[i] * fadeInPercent + _walkingStartAngles[i] * (1 - fadeInPercent);
    }

    publishToServos(angles);

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

    // if walking, update the current pose
    if (_isWalking) {
        walk();
        return;
    }

    // if the trajectory is not finished, follow it
    if (!_trajectory._isFinished) {
        // reset offsets when starting a new trajectory
        _integratedOffsets = Pose();
        _collisionOffsets = Pose();

        follow_trajectory();
        IKResult result = _ikSolver.solveInverseKinematics(_currentPose);

        if (result.success) {
            publishToServos(result.angles);
        }
    }

    // if the trajectory is finished, publish the goal pose
    else {
        // Get current servo angles - now directly use _lastCommandedAngles
        _currentPose = _goalPose + _integratedOffsets + _collisionOffsets;
        vector<float> currentAngles = _lastCommandedAngles;
        // Apply integrated velocity offsets to the goal pose
        Pose _goalPoseWithVelOffset = _currentPose;
    
        vector<float> goalAngles = _ikSolver.solveInverseKinematics(_currentPose).angles;
        

        // Interpolate between current angles and goal angles
        // Move 1% closer to the goal angle each time
        vector<float> interpolatedAngles(6);
        for (int i = 0; i < 6; i++) {
            interpolatedAngles[i] = currentAngles[i] + 0.005 * (goalAngles[i] - currentAngles[i]);
        }

        publishToServos(interpolatedAngles);
        //Serial.println("POSE: " + _goalPoseWithVelOffset.toString());
        //Serial.println("ANGLES: " + String(interpolatedAngles[0]) + ", " + String(interpolatedAngles[1]) + ", " + String(interpolatedAngles[2]) + ", " + String(interpolatedAngles[3]) + ", " + String(interpolatedAngles[4]) + ", " + String(interpolatedAngles[5]));
    }
}
void Controller::startWalking(String direction, float speedMultiplier) {
    _currentPose = Pose(0, 0, 0, 0, 0, 0);
    _trajectory._isFinished = true;
    _isWalking = true;
    _walkingDirection = direction;
    _speedMultiplier = speedMultiplier;
    _walkingStartTime = millis() / 1000.0f;
    _walkingStartAngles = _lastCommandedAngles; 
    _walkingPattern._previousTime = millis() / 1000.0f;
    _walkingPattern._previousTimeInCycle = 0.0;
}

void Controller::setGoalPose(const Pose& goalPose) {
    // check if the goal pose is different from the current pose
    _isWalking = false;

    if (_currentPose == goalPose) {
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

    // check if the current pose is valid by solving the IK for it
    IKResult currentPoseResult = _ikSolver.solveInverseKinematics(_currentPose);

    if (currentPoseResult.success) {
        // if the current pose is valid, then set a trajectory
        set_trajectory(_currentPose, _goalPose);
    } else {
        Serial.println("Current pose is invalid, cannot create trajectory");
    }
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

String Controller::getModeString() {
    switch (_currentMode) {
        case STANDARD:
            return "STANDARD";
        case JOYSTICK_X:
            return "JOYSTICK_X";
        case JOYSTICK_Y:
            return "JOYSTICK_Y";
        case JOYSTICK_Z:
            return "JOYSTICK_Z";
        case JOYSTICK_WALK:
            return "JOYSTICK_WALK";
        default:
            return "UNKNOWN";
    }
}
