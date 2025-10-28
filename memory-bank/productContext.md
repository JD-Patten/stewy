# Product Context: Stewy

## Why This Project Exists

### Problem Statement
Robotics education often faces barriers:
- **Expensive platforms**: Industrial-grade robots cost thousands
- **Oversimplified kits**: Many educational robots don't teach real robotics concepts
- **Complex theory**: Inverse kinematics and control systems seem inaccessible
- **Limited hands-on experience**: Students learn theory but struggle with implementation

### Solution: Stewy
An affordable, approachable platform that teaches real robotics without oversimplification. Users build, program, and experiment with a robot that uses the same mathematical foundations as industrial robots.

## How It Works

### Core Concept: Stewart Platform
A Stewart platform is a parallel robot with 6 legs connecting a base to a top plate. By controlling the leg lengths (via servo angles), the platform achieves 6-DOF motion. Stewy uses this mechanism both for pose control AND walking.

### User Experience Flow

#### 1. Assembly & Setup
- Users 3D print mechanical parts from provided CAD files
- Assemble with kit-provided electronics
- Flash firmware to ESP32 microcontrollers
- Connect to WiFi access point hosted by robot

#### 2. Initial Calibration
- Web interface provides servo offset adjustment
- Users tune each servo to achieve neutral pose
- Set acceleration limits for smooth motion

#### 3. Pose Control (Primary Mode)
- **Web Interface**: 6 input fields (x, y, z, roll, pitch, yaw)
- **Joystick Control** (planned): Physical joystick controls each axis
- Real-time trajectory planning creates smooth motion
- Visual feedback through robot movement

#### 4. Walking Mode
- Three-direction walking: forward, left, right
- Hexagon-shaped buttons on web interface
- Speed multiplier for variable walking speed
- Genetic algorithm-optimized gait pattern

#### 5. Autonomous Behavior (Planned)
- Distance sensor detects obstacles
- Robot retracts head and walks backward
- Demonstrates basic reactive behavior

### Control Modes
The robot cycles through 5 modes via joystick button click:
1. **STANDARD**: Web interface control only
2. **JOYSTICK_X**: Joystick controls X translation
3. **JOYSTICK_Y**: Joystick controls Y translation  
4. **JOYSTICK_Z**: Joystick controls Z (height)
5. **JOYSTICK_WALK**: Joystick controls walking direction/speed

## Educational Value

### Concepts Taught

#### Beginner Level
- Basic embedded programming (Arduino framework)
- Servo motor control
- WiFi communication
- Web interface development

#### Intermediate Level
- Inverse kinematics mathematics
- Coordinate frame transformations
- PID control and trajectory planning
- Sensor integration and filtering

#### Advanced Level
- Multi-variable optimization (walking pattern)
- Parallel mechanism analysis
- Real-time control systems
- Custom behavior development

### Learning Progression
1. **Week 1-2**: Assembly, basic pose control
2. **Week 3-4**: Understanding IK solver, experimenting with poses
3. **Week 5-6**: Walking patterns, gait analysis
4. **Week 7+**: Custom features, behavior programming

## User Interaction Design

### Web Interface
- **Simple and Clean**: Large buttons, clear labels
- **Three Views**:
  - Control view (pose/walking)
  - Settings view (offsets, accelerations)
  - Pose library (saved positions)
- **Real-time**: Immediate response to commands
- **Mobile-friendly**: Works on phones/tablets

### Physical Remote (Head)
- **Ergonomic**: Designed to be held in hand
- **Tactile Feedback**: Physical joystick for precise control
- **Dual Purpose**: Mounts on body or used as remote
- **Wireless**: No tethering during operation

## Success Criteria

### For Users
- Successfully build and calibrate robot
- Understand inverse kinematics fundamentals
- Create custom poses and walking sequences
- Extend with own code modifications

### For Project
- Clear, comprehensive documentation
- Active community of builders
- Educational institution adoption
- Sustainable through kit sales
