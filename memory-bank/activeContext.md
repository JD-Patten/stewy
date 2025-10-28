# Active Context: Stewy

## Current Work Focus

### Primary Goal: Head-Body Integration
The top priority is completing the integration between the detachable head (sensor unit) and body (Stewart platform). The head sends sensor data via UDP; the body needs to respond intelligently to this data.

## Recent Changes

### Communication Module Refactoring ✅
- **What Changed**: Extracted WiFi/HTTP/UDP code into separate `communication.h/cpp` module
- **Why**: Better modularity, cleaner main file, easier to maintain
- **Files Modified**: 
  - Created `communication.h`, `communication.cpp`
  - Modified `stewy.ino` to use callbacks
  - Removed direct WiFi setup from main file
- **Pattern**: Callback-based architecture
  ```cpp
  onHttpCommand(httpCommandHandler);
  onUdpPacket(udpPacketHandler);
  ```

### Web Interface Updates ✅
- **Embedded HTML**: All UI now in `user_interface.h` (PROGMEM)
- **Build Process**: `web_interface/combine.py` generates header from source files
- **Features Added**:
  - Toggle between control/settings/pose-grid views
  - Hexagonal walking direction buttons
  - Servo offset configuration
  - Acceleration limit tuning

## Active Work Areas

### 1. Joystick Pose Control (IN PROGRESS)

**Current State**:
- ✅ Joystick walking control implemented (JOYSTICK_WALK mode)
- ✅ Direction detection via dot product working
- ✅ Speed control from joystick magnitude
- ⏳ Joystick control for individual axes NOT YET IMPLEMENTED

**What Needs to Be Done**:
Implement control for the other 4 modes:
- `JOYSTICK_X`: Joystick Y-axis controls X translation
- `JOYSTICK_Y`: Joystick Y-axis controls Y translation  
- `JOYSTICK_Z`: Joystick Y-axis controls Z (height)
- Additional modes for roll/pitch/yaw control (may need 6th mode or combo)

**Implementation Approach**:
```cpp
// In controller.cpp updateSensorState()
if (_currentMode == JOYSTICK_X) {
    // Map joystick Y to delta X
    float deltaX = _joyY * scaling_factor;
    Pose newGoal = _currentPose;
    newGoal.x += deltaX;
    setGoalPose(newGoal);
}
// Similar for Y, Z, roll, pitch, yaw
```

**Challenges**:
- Need smooth, responsive control
- Avoid rapid setGoalPose calls (trajectory thrashing)
- May need velocity-based control instead of position jumps
- Consider goal velocity instead of goal pose for continuous control

### 2. Collision Avoidance Behavior (NOT STARTED)

**Planned Behavior**:
1. Distance sensor detects obstacle (threshold: e.g., <20cm)
2. Robot retracts head (move Z down? pitch back?)
3. Walk backward for duration
4. Resume normal operation

**Implementation Plan**:
- Add state machine for autonomous behavior
- New mode: `AUTONOMOUS` or behavior flag
- In `updateSensorState()`, check distance threshold
- Trigger pose change (head retraction) then walking
- Override joystick input during autonomous action

**Questions to Resolve**:
- What pose represents "head retracted"?
- How long to walk backward?
- How to smoothly return control to user?
- Should this be a mode or override any mode?

### 3. Head Firmware Integration (PENDING)

**Current State**:
- Head firmware exists but NOT in this repository
- Separate .ino file on ESP32-C3
- Reads HC-SR04 distance sensor
- Reads KY-023 joystick (X, Y, button)
- Sends UDP packets to body

**To Do**:
- Add head firmware to repository (new directory or file)
- Document head calibration process
- Ensure UDP packet struct matches between head/body
- Test head as mounted sensor vs. handheld remote

### 4. Code Cleanup for Public Release (FUTURE)

**Areas Needing Attention**:
- Add comprehensive comments to complex functions
- Document IK mathematics more clearly
- Add examples and tutorials
- README expansion with:
  - Build instructions
  - Calibration guide
  - Usage examples
  - Troubleshooting
- License verification
- Create CONTRIBUTING.md

## Current Design Decisions

### Joystick Centering
- Center values: X=2160, Y=2200
- **Issue**: These values drift with temperature/wear
- **Solution Needed**: Auto-calibration routine or config file

### Walking Speed Control
- Current: Speed multiplier from web UI
- Joystick: Magnitude mapped to speed (0-2000 → speed)
- **Works well** for variable speed walking

### Mode Cycling
- Click joystick button to cycle modes
- **Pros**: Simple, no extra buttons needed
- **Cons**: Can't directly select mode, must cycle through
- **Consider**: Long-press for mode menu? Different button mapping?

## Known Issues Being Tracked

### Issue 1: Joystick Control Modes Not Implemented
- **Status**: High priority, actively working on this
- **Impact**: Can't use joystick for pose control, only walking
- **Blocker**: Need to decide on control scheme (position vs. velocity)

### Issue 2: Servo Jitter at Rest
- **Status**: Mitigated but not solved
- **Workaround**: Small interpolation coefficient (0.005)
- **Root Cause**: Continuous IK solving with floating point variance
- **Potential Fix**: Deadband on pose error before updating servos

### Issue 3: Walking Pattern Limited Directions
- **Status**: By design, acceptable for now
- **Limitation**: Only forward/left/right, no backward
- **Note**: Backward can be achieved with negative speed to forward
- **Future**: Additional patterns for true omni-directional

## Important Patterns & Preferences

### Code Style
- Prefer explicit over implicit
- Use meaningful variable names (not single letters except in math)
- Comment complex algorithms with references
- Keep functions focused (single responsibility)

### Testing Approach
- Serial output for debugging
- Incremental testing of each feature
- Validate IK solutions before commanding
- Test edge cases (limits, zero values, invalid inputs)

### Hardware Considerations
- Always respect servo angle limits
- Validate poses before attempting
- Use acceleration limits to prevent mechanical stress
- Power considerations (6A peak for servos)

## Next Steps (Immediate)

### Step 1: Implement JOYSTICK_X/Y/Z Modes
1. Decide on control scheme (velocity or position delta)
2. Implement in `updateSensorState()`
3. Test each axis independently
4. Tune scaling factors for good feel

### Step 2: Design Roll/Pitch/Yaw Control
1. Determine if existing modes 2-4 should be X/Y/Z or include rotations
2. May need to expand to 6+ modes or use modifier (button held?)
3. Prototype and test

### Step 3: Collision Avoidance Prototype
1. Define behavior state machine
2. Implement distance threshold detection
3. Create head retraction pose
4. Trigger backward walking
5. Test and tune

### Step 4: Head Firmware Integration
1. Add head .ino to repository
2. Document pin assignments
3. Verify UDP packet compatibility
4. Test both mounted and handheld configurations

## Context for Future Sessions

### When Resuming Work
- Check serial monitor for current behavior
- Verify which control mode robot is in
- Test joystick connectivity (UDP packets arriving?)
- Review most recent code changes

### Critical Files for Current Work
- `controller.cpp`: Mode implementation, sensor processing
- `controller.h`: Mode enum, public interface
- `stewy.ino`: Callback handlers, main loop
- `communication.h`: UDP packet structure

### Testing Checklist
- [ ] Basic pose control works (web interface)
- [ ] Walking works (web interface and joystick)
- [ ] Mode cycling works (joystick button)
- [ ] Joystick X/Y/Z modes work (when implemented)
- [ ] Collision avoidance works (when implemented)
- [ ] Head firmware sends valid UDP packets
- [ ] Servo offsets properly calibrated
- [ ] All 6 servos respond correctly

## Learning & Insights

### What Works Well
- **Callback architecture**: Clean separation of concerns
- **IKResult pattern**: Explicit error handling
- **Trajectory planning**: Smooth motion with acceleration limits
- **Walking pattern**: Genetic algorithm optimization produced good gait
- **Web UI**: Intuitive, works on mobile devices

### What Needs Improvement
- **Joystick calibration**: Manual, should be automatic
- **Mode indication**: No visual feedback on current mode
- **Error recovery**: Some failure modes leave robot in bad state
- **Documentation**: Code comments sparse in places
- **Testing**: No automated tests, all manual

### Design Insights
- Stewart platforms are sensitive to IK accuracy
- Real-time control requires careful loop timing
- WiFi latency acceptable for most control tasks
- Physical servo limits more restrictive than expected
- Walking pattern mirroring key to symmetrical gait
