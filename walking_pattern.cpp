#include "walking_pattern.h"


WalkingPattern1::WalkingPattern1() 
    : _x(10.952f, 1.0f, 11.508f, 0.54f)
    , _y(10.476f, 2.0f, -9.921f, 0.365f)
    , _z(25.238f, 1.0f, 89.206f, 0.476f)
    , _roll(19.048f, 1.0f, -4.048f, 0.603f)
    , _pitch(2.54f, 1.0f, 0.714f, 0.286f)
    , _yaw(3.81f, 1.0f, -13.095f, 0.952f)
    , _ikSolver( 50, 100, 73, 15.5, 0,15, 35, 20, -10)
{

}

Pose WalkingPattern1::getPose(float time) const {
    float x = _x.getValue(time);
    float y = _y.getValue(time);
    float z = _z.getValue(time);
    float roll = _roll.getValue(time);
    float pitch = _pitch.getValue(time);
    float yaw = _yaw.getValue(time);
    return Pose{x, y, z, roll, pitch, yaw};
}

vector<float> WalkingPattern1::getAngles(float time, String direction, float speed) const {

    // this walking pattern get's it's movement from a set of sin waves found in the Genetic Algorithm
    // Only a section of the sin waves are used to create the walking pattern, the parts that do most of the work moving the robot
    // This movement is mirrored every other cycle to keep the movement symetrical, and keep the robot walking straight

    
    time = time * speed;

    // get the time in the cycle
    float walkingDuration = 0.5;             // amount of time from the sin waves that we're using in this walking pattern
    float transitionDuration = .1;           // amount of time to transition between the mirrored parts
    float fullcycleDuration = 2 * (walkingDuration + transitionDuration); // account for the time of the mirrored parts
    float timeInCycle = fmod(time, fullcycleDuration); 

    std::vector<float> angles;
    
     if (timeInCycle < walkingDuration) {        // UNMIRRORED WALKING PHASE

        Pose pose = WalkingPattern1::getPose(timeInCycle);
        IKResult result = _ikSolver.solveInverseKinematics(pose);

        // use the angles if the ik solver was successful
        if (result.success) {
            angles = result.angles;
        } else {
            Serial.println("Failed to solve inverse kinematics for walking pattern 1");
            return std::vector<float>();
        }

    } else if (timeInCycle < walkingDuration + transitionDuration) {   // FIRST TRANSITION PHASE

        // linear interpolation between the last servo angles and the first mirrored servo angles
        std::vector<float> startingAngles = _ikSolver.solveInverseKinematics(WalkingPattern1::getPose(walkingDuration)).angles;
        std::vector<float> endingAnglesUnmirrored = _ikSolver.solveInverseKinematics(WalkingPattern1::getPose(0)).angles;
        std::vector<float> endingAngles = {
            -endingAnglesUnmirrored[3],
            -endingAnglesUnmirrored[2],
            -endingAnglesUnmirrored[1],
            -endingAnglesUnmirrored[0],
            -endingAnglesUnmirrored[5],
            -endingAnglesUnmirrored[4]
        };

        float timeInTransition = timeInCycle - walkingDuration;
        vector<float> interpolatedAngles;
        for (int i = 0; i < 6; i++) {
            float interpolatedAngle = startingAngles[i] + (endingAngles[i] - startingAngles[i]) * timeInTransition / transitionDuration;
            interpolatedAngles.push_back(interpolatedAngle);
        }
    
        angles = interpolatedAngles;

    } else if (timeInCycle < walkingDuration + transitionDuration + walkingDuration) {  // MIRRORED WALKING PHASE
        
        float t = timeInCycle - (walkingDuration + transitionDuration);
        Pose pose = WalkingPattern1::getPose(t);
        IKResult result = _ikSolver.solveInverseKinematics(pose);

        // mirror the angles and use them if the ik solver was successful
        if (result.success) {
            angles = {-result.angles[3],
                     -result.angles[2],
                     -result.angles[1],
                     -result.angles[0],
                     -result.angles[5],
                     -result.angles[4]};
        } else {
            Serial.println("Failed to solve inverse kinematics for walking pattern 1");
            return std::vector<float>();
        }
    } else {        // SECOND TRANSITION PHASE

        // linear interpolation between the last mirrored servo angles and the first servo angles
        float timeInTransition = timeInCycle - (walkingDuration + transitionDuration + walkingDuration);

        std::vector<float> startingAnglesUnmirrored = _ikSolver.solveInverseKinematics(WalkingPattern1::getPose(walkingDuration)).angles;
        std::vector<float> endingAngles = _ikSolver.solveInverseKinematics(WalkingPattern1::getPose(0)).angles;

        std::vector<float> startingAngles = {
            -startingAnglesUnmirrored[3],
            -startingAnglesUnmirrored[2],
            -startingAnglesUnmirrored[1],
            -startingAnglesUnmirrored[0],
            -startingAnglesUnmirrored[5],
            -startingAnglesUnmirrored[4]
        };

        std::vector<float> interpolatedAngles;
        for (int i = 0; i < 6; i++) {
            float interpolatedAngle = startingAngles[i] + (endingAngles[i] - startingAngles[i]) * timeInTransition / transitionDuration;
            interpolatedAngles.push_back(interpolatedAngle);
        }
        angles = interpolatedAngles;
    }



     // rearrange the angles for the walking direction and publish them

    if (direction == "right") {
        return angles;
    } else if (direction == "forward") {
        // rearrange the angles for left walking (go from 1,2,3,4,5,6 to 3,4,5,6,1,2)
        return {angles[2], angles[3], angles[4], angles[5], angles[0], angles[1]};
    } else if (direction == "left") {
        // rearrange the angles for right walking (go from 1,2,3,4,5,6 to 5,6,1,2,3,4)
        return {angles[4], angles[5], angles[0], angles[1], angles[2], angles[3]};
    } else {
        Serial.println("Invalid walking direction");
        return {};
    }
}
