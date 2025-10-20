#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <vector>

using namespace Eigen;
using namespace std;

// Define a struct to hold both result and status
struct IKResult {
    vector<float> angles;
    bool success;
    String errorMessage;
    
    // Constructor for success case
    IKResult(vector<float> angles) : angles(angles), success(true) {}
    
    // Constructor for error case
    IKResult(String error) : success(false), errorMessage(error) {
        angles = vector<float>(6, 0.0);  // Initialize with zeros
    }
};

struct Pose {
    float x, y, z, roll, pitch, yaw;

    // Add equality operator
    bool operator==(const Pose& other) const {
        const float epsilon = 0.0001f;  // Small threshold for float comparison
        return (abs(x - other.x) < epsilon &&
                abs(y - other.y) < epsilon &&
                abs(z - other.z) < epsilon &&
                abs(roll - other.roll) < epsilon &&
                abs(pitch - other.pitch) < epsilon &&
                abs(yaw - other.yaw) < epsilon);
    }

    // Add array-like access
    float& operator[](int index) {
        switch(index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            case 3: return roll;
            case 4: return pitch;
            case 5: return yaw;
            default: throw std::out_of_range("Index out of bounds");
        }
    }

    // Const version of array access
    const float& operator[](int index) const {
        switch(index) {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            case 3: return roll;
            case 4: return pitch;
            case 5: return yaw;
            default: throw std::out_of_range("Index out of bounds");
        }
    }

    // Return size of pose (always 6)
    static constexpr int size() { return 6; }

    // Constructor for easy initialization
    Pose(float x = 0, float y = 0, float z = 0, float roll = 0, float pitch = 0, float yaw = 0)
        : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {}

    // construct from a comma-separated String "x,y,z,roll,pitch,yaw"
    Pose(const String &s) {
        // initialize to zeros first
        x = y = z = roll = pitch = yaw = 0.0f;
        // attempt to parse; ignore return value here (keeps zeros on failure)
        fromString(s);
    }

    // Parse a string and set this Pose. Returns true on success (exactly 6 numbers parsed).
    bool fromString(const String &s) {
        // local copy so we can trim/modify
        String in = s;
        // remove leading/trailing spaces:
        auto trim = [](String &str) {
            // trim left
            while (str.length() && isspace((unsigned char)str.charAt(0))) str = str.substring(1);
            // trim right
            while (str.length() && isspace((unsigned char)str.charAt(str.length() - 1))) str = str.substring(0, str.length() - 1);
        };

        trim(in);

        float vals[6] = {0,0,0,0,0,0};
        int lastIndex = 0;
        int commaIndex;
        int count = 0;

        while (count < 6) {
            commaIndex = in.indexOf(',', lastIndex);
            String token;
            if (commaIndex == -1) {
                token = in.substring(lastIndex);
            } else {
                token = in.substring(lastIndex, commaIndex);
            }
            trim(token);                       // remove spaces around number
            if (token.length() == 0) return false; // empty token -> fail

            // convert to float
            vals[count] = token.toFloat();

            count++;
            if (commaIndex == -1) break; // no more commas
            lastIndex = commaIndex + 1;
        }

        // must have parsed exactly 6 numbers and no trailing junk
        if (count != 6) return false;

        // success: assign
        x = vals[0];
        y = vals[1];
        z = vals[2];
        roll = vals[3];
        pitch = vals[4];
        yaw = vals[5];
        return true;
    }


    // Subtraction operator
    Pose operator-(const Pose& other) const {
        return Pose(
            x - other.x,
            y - other.y,
            z - other.z,
            roll - other.roll,
            pitch - other.pitch,
            yaw - other.yaw
        );
    }

    // Addition operator
    Pose operator+(const Pose& other) const {
        return Pose(
            x + other.x,
            y + other.y,
            z + other.z,
            roll + other.roll,
            pitch + other.pitch,
            yaw + other.yaw
        );
    }

    // Addition assignment operator
    Pose& operator+=(const Pose& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        roll += other.roll;
        pitch += other.pitch;
        yaw += other.yaw;
        return *this;
    }

    // multiplication operator (Pose * scalar)
    Pose operator*(float scalar) const {
        return Pose(
            x * scalar,
            y * scalar,
            z * scalar,
            roll * scalar,
            pitch * scalar,
            yaw * scalar
        );
    }
    
    // division operator (Pose / scalar)
    Pose operator/(float scalar) const {
        return Pose(
            x / scalar,
            y / scalar,
            z / scalar,
            roll / scalar,
            pitch / scalar,
            yaw / scalar
        );
    }

    // Convert Pose to String for printing
    String toString() const {
        return String("x: ") + String(x) + 
               " y: " + String(y) + 
               " z: " + String(z) + 
               " roll: " + String(roll) + 
               " pitch: " + String(pitch) + 
               " yaw: " + String(yaw);
    }

    // Function to calculate the magnitude of the position vector
    float magnitude() const {
        return sqrt(x * x + y * y + z * z + roll * roll + pitch * pitch + yaw * yaw);
     }
};

// Free function for scalar * Pose
inline Pose operator*(float scalar, const Pose& pose) {
    return pose * scalar;  // Reuse the member operator*
}

// Inverse Kinematics Solver Class
// this class is used to set up the robot based on it's physical dimensions 
// and then solve for the servo angles based on a desired end-effector position and orientation
class IKSolver {
private:
    // member variables that need to be stored
    float _servoArmLength;
    float _arm2Length;
    
    //derived member variables computed in constructor
    vector<Vector3d> _topPlateArmConnections;
    vector<Vector3d> _servoTranslations;
    vector<Matrix3d> _servoRotations;

    // Helper functions
    vector<Vector3d> computeConnectionPoints(float offset_1, float offset_2, float z_height);
    vector<Matrix3d> initializeServoRotations(float draftAngle);

public:
    // Constructor
    IKSolver(float servoArmLength, float arm2Length, float servoOffset1,
             float servoOffset2, float servoZOffset, float draftAngle, 
             float topPlateOffset1, float topPlateOffset2, float topPlateZOffset);


    // Compute servo angles based on desired end-effector position and orientation
    IKResult solveInverseKinematics(const Pose& pose) const;
};

#endif // INVERSE_KINEMATICS_H
