#include <ArduinoEigenDense.h>
#include <vector>
#include <cmath>
#include "inverse_kinematics.h"

using namespace Eigen;
using namespace std;

// Constructor definition
IKSolver::IKSolver(float servoArmLength, float arm2Length, float servoOffset1,
                   float servoOffset2, float servoZOffset, float draftAngle, 
                   float topPlateOffset1, float topPlateOffset2, float topPlateZOffset)
    // initialize member variables
    : _servoArmLength(servoArmLength)
    , _arm2Length(arm2Length)

    // compute derived member variables
    , _topPlateArmConnections(computeConnectionPoints(topPlateOffset1, topPlateOffset2, topPlateZOffset))
    , _servoTranslations(computeConnectionPoints(servoOffset1, servoOffset2, servoZOffset))
    , _servoRotations(initializeServoRotations(draftAngle))
{
    // constructor body can be empty since all initialization is done in the list
}

// Function to compute servo or connection point coordinates
vector<Vector3d> IKSolver::computeConnectionPoints(float offset_1, float offset_2, float z_height) {
    float d1 = (1.0 / 6.0) * (offset_1 - offset_2);
    float d2 = (1.0 / 3.0) * (offset_1 - offset_2);
    float c = offset_1 - (2 * d1);
    float radius = sqrt(c * c + d2 * d2 - 2 * c * d2 * cos(M_PI / 3.0));

    // Rotation matrices for 120 and 240 degrees around the Z-axis
    Matrix3d rotate_120 = AngleAxisd(2 * M_PI / 3.0, Vector3d::UnitZ()).toRotationMatrix();
    Matrix3d rotate_240 = AngleAxisd(4 * M_PI / 3.0, Vector3d::UnitZ()).toRotationMatrix();

    // Compute translation vectors
    Vector3d number1(-sqrt(radius * radius - (offset_1 / 2) * (offset_1 / 2)), offset_1 / 2, z_height);
    Vector3d number6(number1.x(), -number1.y(), z_height);
    Vector3d number2 = rotate_240 * number6;
    Vector3d number3 = rotate_240 * number1;
    Vector3d number4 = rotate_120 * number6;
    Vector3d number5 = rotate_120 * number1;

    return {number1, number2, number3, number4, number5, number6};
}

// Function to initialize servo rotations
vector<Matrix3d> IKSolver::initializeServoRotations(float draftAngle) {
    vector<float> zAngles = {-120, -120, 0, 0, 120, 120}; // rotate servo to correct side
    vector<float> xAngles = {0, -180, 0, -180, 0, -180};   // flip every other servo 

    vector<Matrix3d> servoRotations;\
    for (int i = 0; i < 6; ++i) {
        Matrix3d rotation =
            AngleAxisd(xAngles[i] * M_PI / 180.0, Vector3d::UnitX()).toRotationMatrix() *
            AngleAxisd(draftAngle * M_PI / 180.0, Vector3d::UnitY()).toRotationMatrix() *
            AngleAxisd(zAngles[i] * M_PI / 180.0, Vector3d::UnitZ()).toRotationMatrix();

        servoRotations.push_back(rotation);
    }
    return servoRotations;
}

// Function to solve inverse kinematics
IKResult IKSolver::solveInverseKinematics(const Pose& pose) {
    vector<float> servoAngles(6, 0.0);

    // Convert roll, pitch, and yaw to radians
    float roll = pose.roll * M_PI / 180.0;
    float pitch = pose.pitch * M_PI / 180.0;
    float yaw = pose.yaw * M_PI / 180.0;

    // Compute rotation matrix
    Matrix3d rotationMatrix = 
        AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix() * 
        AngleAxisd(pitch, Vector3d::UnitY()).toRotationMatrix() * 
        AngleAxisd(roll, Vector3d::UnitX()).toRotationMatrix();

    // Solve inverse kinematics for each arm
    for (int i = 0; i < 6; ++i) {
        // Step 1: Apply rotation and translation to end effector connection point
        Vector3d endConnectionPoint = rotationMatrix * _topPlateArmConnections[i];
        endConnectionPoint = endConnectionPoint + Vector3d(pose.x, pose.y, pose.z);

        // Step 2: Transform point to servo frame
        Vector3d pointInServoFrame = _servoRotations[i] * (endConnectionPoint - _servoTranslations[i]);

        // Step 3: Solve inverse kinematics equation for servo angle
        float px = pointInServoFrame.x();
        float py = pointInServoFrame.y();
        float pz = pointInServoFrame.z();
        float l1 = _servoArmLength;
        float l2 = _arm2Length;

        float denominator = -2 * l1 * sqrt(py * py + pz * pz);
        float angle = 0.0;

        if (denominator == 0) {
            return IKResult("Division by zero in IK computation!");
        }

        float acosValue = (l2 * l2 - px * px - py * py - pz * pz - l1 * l1) / denominator;
        if (acosValue > 1.0 || acosValue < -1.0) {
            return IKResult("IK solution out of range!");
        }

        angle = atan2(pz, py) + ((pz < 0) ? acos(acosValue) : -acos(acosValue));
        angle = angle * 180.0 / M_PI;  // Convert to degrees
        servoAngles[i] = angle;
    }

    return IKResult(servoAngles);  // Return successful result
}
