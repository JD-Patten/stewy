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
2. Command neutral pose (0
