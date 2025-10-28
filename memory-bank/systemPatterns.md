# System Patterns: Stewy

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    HEAD (ESP32-C3)                      │
│  ┌──────────────┐  ┌──────────────┐                    │
│  │  HC-SR04     │  │   KY-023     │                    │
│  │  Distance    │  │   Joystick   │                    │
│  └──────┬───────┘  └──────┬───────┘                    │
│         │                  │                             │
│         └────────┬─────────┘                            │
│                  │                                       │
│         ┌────────▼─────────┐                           │
│         │  Head Firmware   │                           │
│         │  (Separate .ino) │                           │
│         └────────┬─────────┘                           │
│                  │                                       │
│                  │ UDP Packets (One-way)                │
└──────────────────┼───────────────────────────────────┘
                   │ WiFi
                   ▼
┌─────────────────────────────────────────────────────────┐
│              BODY (ESP32-S3-Nano)                       │
│                                                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │            Communication Layer                    │  │
│  │  ┌────────────┐  ┌────────────┐  ┌────────────┐  │  │
│  │  │   WiFi AP  │  │ HTTP Server│  │ UDP Server │  │  │
│  │  │   + mDNS   │  │   (REST)   │  │  (Sensors) │  │  │
│  │  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  │  │
│  └────────┼───────────────┼───────────────┼─────────┘  │
│           │               │               │             │
│  ┌────────▼───────────────▼───────────────▼─────────┐  │
│  │              Main Loop (stewy.ino)               │  │
│  │  - Registers callbacks for HTTP/UDP              │  │
│  │  - Routes commands to Controller                 │  │
│  └────────┬─────────────────────────────────────────┘  │
│           │                                             │
│  ┌────────▼─────────────────────────────────────────┐  │
│  │              Controller                           │  │
│  │  - Trajectory Planning                            │  │
│  │  - Mode Management (5 modes)                      │  │
│  │  - Sensor State Processing                        │  │
│  │  - Walking/Pose Coordination                      │  │
│  └────────┬──────────────────┬──────────────────────┘  │
│           │                  │                          │
│  ┌────────▼──────────┐  ┌───▼────────────────────┐    │
│  │   IK Solver       │  │  Walking Pattern       │    │
│  │  - Pose → Angles  │  │  - Sin Wave Generator  │    │
│  │  - Eigen Math     │  │  - Gait Mirroring      │    │
│  └────────┬──────────┘  └───┬────────────────────┘    │
│           │                  │                          │
│           └────────┬─────────┘                          │
│                    │                                     │
│           ┌────────▼──────────┐                         │
│           │  Servo Manager    │                         │
│           │  - 6 PWM Channels │                         │
│           │  - Angle Offsets  │                         │
│           │  - Limits/Clamp   │                         │
│           └────────┬──────────┘                         │
│                    │                                     │
│  ┌─────────────────▼──────────────────────────────┐    │
│  │         6x Servo Motors (Hardware)              │    │
│  └─────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

## Key Components

### 1. Inverse Kinematics Solver (`IKSolver` class)

**Purpose**: Convert desired end-effector pose (x,y,z,roll,pitch,yaw) to 6 servo angles

**Algorithm**:
```
For each of 6 legs:
1. Apply rotation matrix to top plate connection point
2. Add translation to get world coordinates
3. Transform to servo's local frame
4. Solve 2-link inverse kinematics equation
5. Convert from radians to degrees
```

**Key Design Decisions**:
- Uses Eigen library for matrix operations
- Precomputes connection points and rotation matrices in constructor
- Returns `IKResult` struct with success flag and error messages
- Supports arbitrary Stewart platform geometry via constructor parameters

**Critical Parameters** (in stewy.ino):
- `servoArmLength`: 50mm (first link)
- `arm2Length`: 100mm (second link)
- `servoOffset1/2`: Base plate arm spacing
- `topPlateOffset1/2`: Top plate arm spacing
- `draftAngle`: 15° (servo tilt angle)

### 2. Controller (`Controller` class)

**Purpose**: Orchestrate all motion control, sensor processing, mode management

**State Machine**:
```
Modes (cycle on joystick click):
STANDARD → JOYSTICK_X → JOYSTICK_Y → JOYSTICK_Z → JOYSTICK_WALK → (back to STANDARD)
```

**Key Responsibilities**:
1. **Trajectory Planning**: 3rd order polynomial time scaling (Modern Robotics textbook)
2. **Mode Management**: Handle 5 different control modes
3. **Sensor Processing**: Joystick deadzone, centering, direction detection
4. **Walking Coordination**: Fade-in transitions, pattern mirroring
5. **Servo Management**: Apply offsets, enforce limits

**Important Methods**:
- `update()`: Main control loop, called every iteration
- `setGoalPose()`: Validates and initiates new pose trajectory
- `startWalking()`: Transitions from pose control to walking
- `updateSensorState()`: Processes joystick/distance sensor data

**Trajectory Planning**:
- Minimizes duration while respecting acceleration limits
- Separate limits for translation (`maxAcceleration`) and rotation (`maxAngularAcceleration`)
- Uses s-curve (3rd order polynomial) for smooth motion
- Formula: `s(t) = 3t²/T² - 2t³/T³` where T is duration

### 3. Walking Pattern (`WalkingPattern1` class)

**Algorithm**: Genetic algorithm-optimized sin wave composition

**IMPORTANT - Genetic Algorithm Origin**:
- The sin wave parameters were generated by a genetic algorithm that optimized ONLY for distance traveled
- The algorithm did NOT optimize for direction of travel
- The resulting pattern inherently moves the robot BACKWARD by default
- To correct this, the speed parameter is negated at the start of `getAngles()`: `speed = -speed;`
- This reverses the animation playback, making forward motion actually move forward
- The mirroring pattern (explained below) keeps the robot walking straight, but does not guarantee forward motion
- Future walking patterns from genetic algorithms may also need velocity negation if they move backward

**6 Sin Waves** (one per DOF):
```cpp
x:     amplitude=10.952, period=1.0, offset=11.508,  phase=0.54
y:     amplitude=10.476, period=2.0, offset=-9.921,  phase=0.365
z:     amplitude=25.238, period=1.0, offset=89.206,  phase=0.476
roll:  amplitude=19.048, period=1.0, offset=-4.048,  phase=0.603
pitch: amplitude=2.54,   period=1.0, offset=0.714,   phase=0.286
yaw:   amplitude=3.81,   period=1.0, offset=-13.095, phase=0.952
```

**Gait Pattern**:
```
Full Cycle: 1.4s (adjustable with speed multiplier)
├─ Phase 1 (0.5s): Forward motion using sin waves
├─ Transition 1 (0.1s): Linear interpolation
├─ Phase 2 (0.5s): Mirrored motion (swap legs, negate angles)
└─ Transition 2 (0.1s): Linear interpolation back
```

**Mirroring Pattern**: `[1,2,3,4,5,6] → [-4,-3,-2,-1,-6,-5]`

**Direction Mapping**:
- Forward: Rotate leg assignment by 2 positions
- Left: Rotate leg assignment by 4 positions
- Right: No rotation (base pattern)

### 4. Communication Layer

**WiFi Configuration**:
- Access Point mode (no internet required)
- SSID: "stewyAP", Password: "stewy123"
- mDNS: `robot.local` (for easier connection)
- UDP Port: 4210 (sensor data)
- HTTP Port: 80 (web interface)

**Protocol Design**:
- **HTTP**: Command-based REST API
  - `/pose1?params=x,y,z,roll,pitch,yaw`
  - `/walk?params=speed,direction`
  - `/setOffsets?params=s1,s2,s3,s4,s5,s6`
- **UDP**: Real-time sensor packets (6 bytes)
  ```
  struct JoystickPacket {
    int16_t joyX;      // 2 bytes
    int16_t joyY;      // 2 bytes
    uint8_t clicked;   // 1 byte
    uint8_t padding;   // 1 byte
    uint16_t distance; // 2 bytes
  }
  ```

**Callback Pattern**:
```cpp
// Main program registers handlers
onHttpCommand(httpCommandHandler);
onUdpPacket(udpPacketHandler);

// Communication module calls them when data arrives
```

## Critical Design Patterns

### 1. Servo Angle Sign Convention
**IMPORTANT**: Odd-numbered servos (1,3,5) have inverted angles
```cpp
// When publishing to servos:
for (i = 0; i < 6; i++) {
    checkAngle = (i % 2 == 0) ? -angles[i] : angles[i];
    // Apply offset, limits...
    publishAngle = (i % 2 == 0) ? -constrainedAngle : constrainedAngle;
    servos[i].write(publishAngle + 90);  // 90° is neutral
}
```

### 2. Joystick Direction Detection
Uses **dot product** with unit vectors to find best match:
```cpp
directions = {
    "forward": (0, 1.0),
    "left": (0.866, -0.5),     // 120°
    "right": (-0.866, -0.5),   // 240°
    // ... etc
}

// Find direction with largest dot product
dot = joyX * dirX + joyY * dirY
```

### 3. Walking Fade-In
Prevents jarring transitions when starting to walk:
```cpp
fadeInDuration = 0.2 / abs(speed);
fadeInPercent = min(timeSinceStart / fadeInDuration, 1.0);
finalAngle = walkingAngle * fadeInPercent + startAngle * (1 - fadeInPercent);
```

### 4. Pose Validation
Always validate IK solution before accepting new goal:
```cpp
IKResult result = ikSolver.solveInverseKinematics(goalPose);
if (!result.success) {
    // Reject pose, keep current goal
    return;
}
```

## Data Flow Examples

### Web Command → Servo Motion
```
1. User clicks "Set Pose" on web interface
2. JavaScript sends HTTP GET: /pose1?params=0,0,110,0,0,0
3. Communication module parses request
4. Calls registered httpCommandHandler()
5. Handler parses params into Pose object
6. controller.setGoalPose(pose)
7. Controller validates with IK solver
8. Creates trajectory from current → goal
9. Main loop calls controller.update()
10. Controller follows trajectory, solves IK each step
11. Publishes angles to servos
```

### Joystick → Walking
```
1. Head firmware reads joystick (joyX, joyY, clicked)
2. Sends UDP packet to body
3. Communication module receives packet
4. Calls registered udpPacketHandler()
5. Handler calls controller.updateSensorState()
6. Controller detects mode = JOYSTICK_WALK
7. Computes direction via dot product
8. Calls startWalking(direction, speed)
9. Main loop calls controller.update()
10. Controller calls walkingPattern.getAngles()
11. Publishes angles to servos
```

## Performance Characteristics

- **Control Loop Rate**: ~100Hz (limited by servo update rate)
- **Trajectory Resolution**: Continuous (evaluated each loop)
- **Walking Cycle**: 1.4s at speed=1.0 (variable with speed multiplier)
- **IK Solve Time**: <1ms per pose
- **WiFi Latency**: ~10-50ms (UDP packets)
- **Servo Update Rate**: 50Hz (standard PWM frequency)

## Extension Points

### Adding New Walking Patterns
1. Create new class inheriting pattern interface
2. Implement `getAngles(direction, speed)` method
3. Initialize in Controller constructor
4. Switch between patterns via mode or command

### Adding Control Modes
1. Add enum value to `ControlMode`
2. Increment modulo in mode cycling logic
3. Implement behavior in `updateSensorState()` or `update()`

### Custom Commands
1. Add route handler in stewy.ino `httpCommandHandler()`
2. Parse parameters from URL string
3. Call appropriate Controller methods
