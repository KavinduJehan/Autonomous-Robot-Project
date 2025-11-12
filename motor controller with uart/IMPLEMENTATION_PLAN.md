# ESP32 Navigation Integration - Implementation Plan

## Overview
Integrate ESP32 navigation module with Raspberry Pi motor controller for autonomous navigation with dynamic obstacle avoidance.

## Architecture

```
ESP32 (Navigator)  →  Raspberry Pi (Orchestrator)  →  STM32 (Motor Controller)
     ↓                         ↓                              ↓
- Path planning          - Command arbitration          - Motor execution
- Shortest path          - YOLO processing              - Wall avoidance (PID)
- Turn-by-turn           - Obstacle detection           - Emergency stop
- WiFi/Serial comm       - Traffic sign recognition     - Ultrasonic safety
```

## Communication Protocol

### ESP32 → RPi (Navigation Commands)
**Format:** JSON over Serial/WiFi
```json
{
  "type": "navigation",
  "command": "TURN_LEFT",
  "angle": 90,
  "distance": 0,
  "timestamp": 1234567890,
  "priority": "normal"
}
```

**Command Set:**
- `MOVE_FORWARD` - Move forward X meters
- `TURN_LEFT` - Turn left X degrees
- `TURN_RIGHT` - Turn right X degrees
- `STOP` - Stop navigation
- `REACHED_WAYPOINT` - Waypoint reached notification
- `REACHED_DESTINATION` - Final destination reached
- `RECALCULATING` - Path recalculation in progress

### RPi → STM32 (Motor Commands)
**Format:** Single ASCII byte (existing protocol)
- `F` - Forward
- `R` - Reverse
- `L` - Turn left
- `T` - Turn right
- `S` - Stop

### RPi → ESP32 (Status/Feedback)
**Format:** JSON over Serial/WiFi
```json
{
  "type": "status",
  "state": "executing",
  "obstacle_detected": false,
  "current_position": {"x": 1.5, "y": 2.3},
  "heading": 90,
  "ultrasonic_left": 25.5,
  "ultrasonic_right": 30.2
}
```

## Implementation Phases

### Phase 1: ESP32-RPi Communication (Week 1)
**Goal:** Establish reliable bidirectional communication

#### Step 1.1: Choose Communication Method
**Option A: Serial UART (Recommended for simplicity)**
```
ESP32 UART1 ↔ RPi USB-Serial adapter (/dev/ttyUSB0)
- Baud: 115200 (faster than STM32 link)
- Format: JSON messages with newline delimiter
```

**Option B: WiFi Socket (Recommended for flexibility)**
```
ESP32 (TCP Server) ↔ RPi (TCP Client)
- Port: 8888
- Protocol: JSON over TCP
- Allows wireless debugging and monitoring
```

#### Step 1.2: ESP32 Code Setup
**File:** `esp32_navigator.ino`
```cpp
// Send navigation commands to RPi
void sendNavigationCommand(String cmd, int angle, float distance) {
  StaticJsonDocument<256> doc;
  doc["type"] = "navigation";
  doc["command"] = cmd;
  doc["angle"] = angle;
  doc["distance"] = distance;
  doc["timestamp"] = millis();
  
  serializeJson(doc, Serial);
  Serial.println();
}

// Example usage:
sendNavigationCommand("TURN_LEFT", 90, 0);
sendNavigationCommand("MOVE_FORWARD", 0, 2.5);
```

#### Step 1.3: RPi Code - Navigation Module
**File:** `navigation_interface.py`
- Receive commands from ESP32
- Parse JSON
- Translate to STM32 commands
- Send feedback to ESP32

#### Step 1.4: Integration Testing
- [ ] ESP32 sends test commands
- [ ] RPi receives and logs commands
- [ ] RPi sends status back to ESP32
- [ ] Verify message integrity

---

### Phase 2: Command Arbitration System (Week 2)
**Goal:** Intelligent command prioritization and conflict resolution

#### Step 2.1: Command Priority System
```python
Priority Levels:
1. EMERGENCY (highest)    - YOLO detects obstacle, emergency stop
2. SAFETY                 - Ultrasonic wall collision (STM32 handles)
3. DYNAMIC_OBSTACLE       - YOLO detects obstacle, plan around
4. NAVIGATION            - ESP32 turn-by-turn commands
5. MANUAL (lowest)        - User override via keyboard/UI
```

#### Step 2.2: State Machine
```
States:
- IDLE: Waiting for commands
- NAVIGATING: Following ESP32 route
- AVOIDING_OBSTACLE: YOLO detected obstacle, temporary detour
- EMERGENCY_STOP: Critical obstacle, stopped
- MANUAL_OVERRIDE: User control active
```

#### Step 2.3: Arbitrator Module
**File:** `command_arbitrator.py`
- Queue management
- Priority scheduling
- Timeout handling
- Conflict resolution

---

### Phase 3: Navigation Execution (Week 3)
**Goal:** Execute ESP32 commands with feedback

#### Step 3.1: Navigation Executor
**Features:**
- Convert angles to timed turns
- Convert distances to timed movements
- Monitor execution completion
- Send progress updates to ESP32

#### Step 3.2: Odometry Estimation
```python
# Simple dead reckoning
- Track wheel rotations (if encoder available)
- Estimate distance traveled
- Estimate heading changes
- Report position to ESP32
```

#### Step 3.3: Waypoint Management
- Track current waypoint
- Detect waypoint arrival
- Request next command from ESP32

---

### Phase 4: YOLO Integration Preparation (Week 4)
**Goal:** Prepare for obstacle detection (Phase 2 of your plan)

#### Step 4.1: Camera Setup
- Install Pi Camera / USB Camera
- Test frame capture
- Verify YOLO model loading

#### Step 4.2: Obstacle Detection Hooks
```python
# Placeholder for YOLO integration
def check_dynamic_obstacles():
    # Will detect: people, vehicles, animals
    # Return: obstacle_present, obstacle_type, distance
    pass
```

#### Step 4.3: Traffic Sign Recognition Hooks
```python
# Placeholder for sign detection
def detect_traffic_signs():
    # Will detect: stop, yield, speed limit
    # Return: sign_type, confidence
    pass
```

---

## Hardware Connections

### Current Setup
```
RPi GPIO 14 (TX) → STM32 PA10 (RX)  [UART 9600 baud]
RPi GPIO 15 (RX) → STM32 PA9  (TX)
```

### New Additions
**Option A: ESP32 via USB-Serial**
```
ESP32 TX → USB-Serial RX → RPi /dev/ttyUSB0  [115200 baud]
ESP32 RX → USB-Serial TX
```

**Option B: ESP32 via WiFi**
```
ESP32 WiFi → Router → RPi WiFi
(No physical connection needed)
```

---

## Software Architecture

### File Structure
```
motor controller with uart/
├── rpi_motor_controller.py          # Existing STM32 interface
├── navigation_interface.py           # NEW: ESP32 communication
├── command_arbitrator.py             # NEW: Command prioritization
├── navigation_executor.py            # NEW: Execute nav commands
├── obstacle_detector.py              # NEW: YOLO integration (Phase 2)
├── traffic_sign_detector.py          # NEW: Sign recognition (Phase 2)
├── main_orchestrator.py              # NEW: Main control loop
├── config.yaml                       # NEW: Configuration
└── tests/
    ├── test_esp32_comm.py
    ├── test_arbitrator.py
    └── test_navigation.py
```

---

## Testing Strategy

### Unit Tests
- [ ] ESP32 message parsing
- [ ] Command priority sorting
- [ ] Navigation command execution
- [ ] Timeout handling

### Integration Tests
- [ ] ESP32 → RPi → STM32 full chain
- [ ] Obstacle override during navigation
- [ ] Emergency stop propagation
- [ ] Waypoint completion feedback

### System Tests
- [ ] Navigate simple path (3 waypoints)
- [ ] Navigate with manual obstacle placement
- [ ] Test WiFi disconnection recovery
- [ ] Test ESP32 reset recovery

---

## Configuration File Example

**config.yaml**
```yaml
esp32:
  connection_type: "wifi"  # or "serial"
  
  # WiFi settings
  wifi:
    host: "192.168.1.100"
    port: 8888
    reconnect_interval: 5
  
  # Serial settings
  serial:
    port: "/dev/ttyUSB0"
    baudrate: 115200
  
  timeout: 5.0

stm32:
  port: "/dev/serial0"
  baudrate: 9600
  heartbeat_interval: 1.5

navigation:
  turn_duration_per_degree: 0.01  # seconds per degree
  forward_speed: 70  # PWM percentage
  turn_speed: 60
  position_tolerance: 0.1  # meters
  angle_tolerance: 5  # degrees

safety:
  emergency_stop_distance: 0.3  # meters
  slow_down_distance: 1.0
  max_command_age: 10.0  # seconds
  require_ultrasonic: true

logging:
  level: "INFO"
  file: "robot.log"
  console: true
```

---

## Development Milestones

### Milestone 1: Basic Communication (3 days)
- [x] Choose ESP32 communication method
- [ ] Implement ESP32 sender code
- [ ] Implement RPi receiver code
- [ ] Test bidirectional communication
- [ ] Handle connection errors

### Milestone 2: Command System (4 days)
- [ ] Design command protocol
- [ ] Implement command parser
- [ ] Create arbitrator module
- [ ] Test priority system
- [ ] Add timeout handling

### Milestone 3: Navigation Execution (5 days)
- [ ] Implement navigation executor
- [ ] Calibrate turn durations
- [ ] Calibrate forward distances
- [ ] Test waypoint navigation
- [ ] Add feedback to ESP32

### Milestone 4: Integration & Testing (3 days)
- [ ] Full system integration
- [ ] End-to-end testing
- [ ] Performance tuning
- [ ] Documentation
- [ ] Demo preparation

---

## Next Steps (START HERE)

### Immediate Actions:
1. **Choose Communication Method:**
   - WiFi (flexible, easier debugging)
   - Serial (simpler, lower latency)

2. **Set up ESP32:**
   - Install Arduino IDE / PlatformIO
   - Install ArduinoJson library
   - Test serial/WiFi output

3. **Create Navigation Interface:**
   - Start with `navigation_interface.py`
   - Parse simple JSON commands
   - Log received commands

4. **First Test:**
   - ESP32 sends: `{"command": "TURN_LEFT", "angle": 90}`
   - RPi receives and logs it
   - RPi sends: `L` to STM32
   - Robot turns left

---

## Future Enhancements (Post-MVP)

- [ ] GPS integration for absolute positioning
- [ ] IMU for accurate heading
- [ ] Wheel encoders for precise odometry
- [ ] Multi-robot coordination
- [ ] Web dashboard for monitoring
- [ ] ROS integration
- [ ] Autonomous charging station return

---

## Questions to Answer

1. **Communication:** WiFi or Serial for ESP32-RPi?
2. **ESP32 capabilities:** Does it have GPS? IMU?
3. **Environment:** Indoor or outdoor navigation?
4. **Map format:** How does ESP32 represent paths?
5. **Camera:** Already have camera for YOLO?

