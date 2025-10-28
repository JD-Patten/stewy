# Technical Context: Stewy

## Technology Stack

### Hardware Platform

#### Body Microcontroller
- **Model**: Waveshare ESP32-S3-Nano
- **Processor**: Dual-core Xtensa LX7 @ 240MHz
- **RAM**: 512KB SRAM
- **Flash**: 8MB
- **WiFi**: 802.11 b/g/n (2.4 GHz)
- **GPIO**: Sufficient for 6 PWM channels + peripherals

#### Head Microcontroller
- **Model**: ESP32-C3 Supermini
- **Processor**: RISC-V single-core @ 160MHz
- **RAM**: 400KB SRAM
- **Flash**: 4MB
- **WiFi**: 802.11 b/g/n (2.4 GHz)
- **Purpose**: Lightweight sensor reading and UDP transmission

#### Actuators
- **Type**: 9g metal gear servo motors
- **Quantity**: 6
- **Control**: PWM (50Hz standard)
- **Range**: 0-180° (software limited to ±80° from neutral)
- **Pins**: GPIO 3, 2, 1, 18, 17, 10 (body)

#### Sensors (Head)
- **HC-SR04 Ultrasonic Distance Sensor**
  - Range: 2cm - 400cm
  - Trigger/Echo pins on ESP32-C3
  - Purpose: Obstacle detection
  
- **KY-023 Dual-Axis Joystick**
  - 2x Analog outputs (ADC)
  - 1x Digital button
  - Center values: X=2160, Y=2200 (needs calibration)
  - Deadzone: ±25 units

### Software Stack

#### Development Framework
- **Core**: Arduino Framework
- **Platform**: PlatformIO or Arduino IDE
- **Language**: C++ (C++11 minimum)
- **Build System**: Arduino build chain

#### Key Libraries

##### ESP32Servo
- **Purpose**: PWM servo control on ESP32
- **Features**: 
  - Multiple timer allocation
  - Microsecond pulse width control
  - Configurable min/max pulse (500-2500μs)
- **Usage**: 
  ```cpp
  ESP32PWM::allocateTimer(0-3);  // Allocate 4 timers
  servo.setPeriodHertz(50);
  servo.attach(pin, 500, 2500);
  ```

##### ArduinoEigenDense
- **Purpose**: Linear algebra for IK calculations
- **Features**:
  - Matrix operations
  - Rotation matrices (AngleAxis)
  - Vector math
- **Size Impact**: ~50KB of flash
- **Performance**: Optimized for embedded
- **Usage**:
  ```cpp
  Matrix3d rotation = AngleAxisd(angle, Vector3d::UnitZ()).toRotationMatrix();
  Vector3d result = rotation * point;
  ```

##### WiFi (ESP32 Core)
- **Mode**: Access Point (softAP)
- **Features**:
  - DHCP server built-in
  - WPA2 security
  - No internet connectivity needed
- **Configuration**:
  ```cpp
  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();
  ```

##### WebServer (ESP32 Core)
- **Purpose**: HTTP REST API
- **Features**:
  - Route handling
  - GET parameter parsing
  - Static content serving
- **Limitations**: Single-threaded, blocking

##### WiFiUDP (ESP32 Core)
- **Purpose**: Real-time sensor data
- **Port**: 4210
- **Packet Size**: 6 bytes (JoystickPacket struct)
- **Performance**: ~10-50ms latency

##### ESPmDNS (ESP32 Core)
- **Purpose**: DNS-SD for easier connection
- **Hostname**: `robot.local`
- **Benefit**: Users don't need to remember IP address

### File Organization

```
stewy/
├── stewy.ino                    # Main entry point
├── controller.h / .cpp          # Motion control
├── inverse_kinematics.h / .cpp  # IK solver
├── walking_pattern.h / .cpp     # Gait generation
├── communication.h / .cpp       # WiFi/HTTP/UDP
├── user_interface.h             # Embedded HTML/CSS/JS
├── README.md                    # Project overview
├── LICENSE                      # Open source license
├── custom_instructions.clinerules # Cline memory bank config
├── CAD/                         # 3D printable parts
│   ├── Stewy Arm Left.step
│   ├── Stewy Arm Right.step
│   ├── Stewy Base Bottom.step
│   ├── Stewy Base top.step
│   └── Stewy Top Plate.step
├── web_interface/               # Web UI development
│   ├── index.html
│   ├── styles.css
│   ├── script.js
│   └── combine.py              # Build tool for user_interface.h
└── memory-bank/                # Cline context (this documentation)
    ├── projectbrief.md
    ├── productContext.md
    ├── systemPatterns.md
    ├── techContext.md
    ├── activeContext.md
    └── progress.md
```

## Development Setup

### Required Tools
1. **Arduino IDE** (v2.0+) OR **PlatformIO** (recommended)
2. **USB Cable** (USB-C for ESP32-S3-Nano)
3. **Serial Monitor** (for debugging)
4. **Python 3** (for web interface build script)

### Board Setup (Arduino IDE)
1. Install ESP32 board package:
   - URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Select Board: "ESP32S3 Dev Module" (for body)
3. Select Port: Appropriate COM/tty port

### Library Installation
```
Required Libraries (Arduino Library Manager):
- ESP32Servo (by Kevin Harrington)
- ArduinoEigen (search "Eigen")

Built-in (no install needed):
- WiFi
- WebServer
- WiFiUDP
- ESPmDNS
```

### Build Process

#### For Body Firmware
1. Open `stewy.ino` in IDE
2. Select ESP32-S3 board
3. Compile and upload
4. Monitor serial output for IP address

#### For Web Interface
```bash
cd web_interface/
python combine.py
# This generates user_interface.h from index.html, styles.css, script.js
```

### Debugging Workflow

#### Serial Debugging
```cpp
Serial.begin(115200);
Serial.println("Debug message");
Serial.print("Value: "); Serial.println(value);
```

#### WiFi Connection Testing
1. Connect device to "stewyAP" (password: stewy123)
2. Navigate to `http://192.168.4.1` or `http://robot.local`
3. Check browser console for JavaScript errors

#### Servo Calibration
1. Set all offsets to 0
2. Command neutral pose (0,0,130,0,0,0)
3. Measure physical deviation per servo
4. Input offsets in web UI
5. Save to code via `/setOffsets` endpoint

## Technical Constraints

### Memory Limitations
- **Flash**: 8MB available, ~200KB used by code
- **RAM**: 512KB available, ~50KB used at runtime
- **Heap**: Must avoid fragmentation, use stack where possible
- **Eigen Library**: Statically allocated matrices preferred

### Processing Constraints
- **Control Loop**: Must complete in <10ms for smooth motion
- **IK Solving**: ~1ms per solve (acceptable)
- **WiFi Overhead**: Don't block main loop waiting for packets
- **Floating Point**: ESP32 has FPU, use freely

### Mechanical Constraints
- **Servo Angle Limits**: ±80° from neutral (90°)
  - Physical collision beyond this range
  - Software enforced in `publishToServos()`
- **Acceleration Limits**: 
  - Translation: Tunable (default 10 mm/s²)
  - Rotation: Tunable (default 10 deg/s²)
  - Prevents servo stall and mechanical stress

### Power Considerations
- **Servos**: ~1A peak per servo (6A max)
- **ESP32**: ~200mA active WiFi
- **Recommended**: 5V 10A power supply
- **Battery**: 2S LiPo (7.4V) with 5V BEC possible

### WiFi Range
- **Typical**: 10-30 meters indoor
- **Factors**: Walls, interference, antenna orientation
- **Mitigation**: Stay within visual range for safety

## Configuration Parameters

### Servo Calibration (in stewy.ino)
```cpp
// Metal gear offsets (current)
vector<float> servoOffsets = {0, -2, 7, -4, 4, 0};

// Plastic gear offsets (commented out)
// vector<float> servoOffsets = {5, -3, 2, -2, 8, 6};

// Servo pins
int servoPins[6] = {3, 2, 1, 18, 17, 10};
```

### IK Solver Parameters (in stewy.ino)
```cpp
IKSolver ikSolver(
    50,     // servoArmLength (mm)
    100,    // arm2Length (mm)
    73,     // servoOffset1 (base plate)
    15.5,   // servoOffset2 (base plate)
    0,      // servoZOffset
    15,     // draftAngle (degrees)
    40,     // topPlateOffset1
    20,     // topPlateOffset2
    -10     // topPlateZOffset
);
```

### Network Configuration (in stewy.ino)
```cpp
const char *ssid = "stewyAP";
const char *password = "stewy123";
// UDP port defined in communication.cpp: 4210
```

### Controller Tuning (in stewy.ino)
```cpp
float maxAcceleration = 10;          // mm/s²
float maxRotationalAcceleration = 10; // deg/s²
```

### Joystick Calibration (in controller.cpp)
```cpp
float xCenter = 2160;
float yCenter = 2200;
float deadzone = 25;
```

## Common Development Patterns

### Adding a New HTTP Command
```cpp
// In stewy.ino httpCommandHandler():
if (path == "/myCommand") {
    // Parse params
    // Call controller methods
    // Respond
}
```

### Modifying Walking Pattern
```cpp
// Option 1: Tune existing sin waves in walking_pattern.cpp constructor
// Option 2: Create new WalkingPattern class with different algorithm
```

### Adding Sensor Data to UDP Packet
```cpp
// 1. Modify struct in communication.h
// 2. Update head firmware to send new data
// 3. Update udpPacketHandler in stewy.ino
// 4. Process in controller.updateSensorState()
```

### Custom Pose Sequences
```cpp
// In stewy.ino or new module:
vector<Pose> sequence = {
    Pose(0, 0, 130, 0, 0, 0),
    Pose(10, 0, 120, 5, 0, 0),
    // ... more poses
};

for (auto& pose : sequence) {
    controller.setGoalPose(pose);
    while (!controller._trajectory._isFinished) {
        controller.update();
        delay(10);
    }
}
```

## Known Issues & Workarounds

### Issue: Servos jitter at goal pose
**Cause**: Continuous IK solving with floating point error  
**Workaround**: Interpolation with small coefficient (0.005) in `update()`

### Issue: Web interface doesn't load
**Cause**: User interface HTML too large, out of PROGMEM  
**Workaround**: Keep HTML minimal, inline CSS/JS

### Issue: Walking direction inconsistent
**Cause**: Joystick centering values drift  
**Workaround**: Recalibrate `xCenter`, `yCenter` in controller.cpp

### Issue: IK solver fails for valid poses
**Cause**: Numerical precision in acos() domain check  
**Workaround**: Already handled with range check in inverse_kinematics.cpp

## Future Technical Considerations

### Planned Enhancements
- Head firmware integration into this repository
- OTA (Over-The-Air) firmware updates
- Persistent configuration (save offsets to EEPROM)
- Collision avoidance behavior implementation
- Multi-pattern switching via web UI

### Scalability
- Code can support additional sensors via UDP packet expansion
- Walking pattern framework allows multiple gaits
- HTTP API can be extended for new commands
- Web UI has toggle system for additional views
