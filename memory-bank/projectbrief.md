# Project Brief: Stewy

## Project Overview
Stewy is an educational Stewart platform robot designed to teach robotics concepts at multiple skill levels. The robot features a hexapod-like body with 6 degrees of freedom (6-DOF) for pose control and walking locomotion capabilities.

## Core Mission
Create an accessible, open-source robotics learning platform that teaches:
- Inverse kinematics and forward kinematics
- Control systems and trajectory planning
- Embedded programming on ESP32 microcontrollers
- Real-time sensor integration
- WiFi communication protocols

## Hardware Architecture

### Body (Stewart Platform)
- **Microcontroller**: Waveshare ESP32-S3-Nano
- **Actuators**: 6x 9g metal gear servo motors
- **Mechanical**: Stewart platform configuration with 6-DOF top plate
- **Purpose**: Main control unit, executes motion control

### Head (Sensor Unit - Detachable)
- **Microcontroller**: ESP32-C3 Supermini
- **Sensors**:
  - HC-SR04 ultrasonic distance sensor
  - KY-023 dual-axis joystick module
- **Purpose**: Sensor input, acts as mounted head OR handheld remote
- **Communication**: One-way WiFi/UDP to body (head → body only)

### CAD Files
- 3D printable parts for user fabrication
- Located in `/CAD/` directory
- Users can modify designs for custom builds

## Business Model
- **Original Plan**: Kickstarter campaign (failed)
- **Current Plan**: Direct sales model
- **Product Format**: Kits with all electronic/mechanical components
- **Software**: Fully open-source

## Target Audience
Multi-level educational tool:
- **Beginners**: Learn basic robotics and programming concepts
- **Intermediate**: Understand kinematics and control theory
- **Advanced**: Extend with custom walking patterns, new features

## Key Features
1. **6-DOF Pose Control**: Full position (x,y,z) and orientation (roll,pitch,yaw)
2. **Walking Locomotion**: Tripod gait for three directions (forward, left, right)
3. **Dual Control Modes**:
   - Web interface for pose/walking control
   - Physical joystick on detachable head/remote
4. **Autonomous Behaviors**: Collision avoidance using distance sensor
5. **Extensible Design**: Framework for additional walking patterns

## Project Goals
- Code cleanup and documentation for public release
- Complete head-body integration
- Implement full joystick control for all 6 pose axes
- Collision avoidance behavior
- Educational materials and tutorials
