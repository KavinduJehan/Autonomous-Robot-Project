# Autonomous Robot Development Plan

## ✅ Phase 1 — STM32 Motor Controller (COMPLETED)
*Objective:* Create reliable motor control with collision avoidance using STM32F401RC.

*Completed Tasks:*
- ✅ STM32 development environment setup
- ✅ MX1508 motor driver integration with PWM control
- ✅ UART communication interface (9600 baud)
- ✅ HC-SR04 ultrasonic sensors (Left, Right, Front)
- ✅ Real-time collision avoidance system
- ✅ FreeRTOS-based multitasking architecture
- ✅ Safety systems with emergency stops
- ✅ Status LED indicators

*Deliverable:* ✅ STM32 motor controller with 3-sensor collision avoidance

---

# Autonomous Robot Development Plan

## ✅ Phase 1 — STM32 Motor Controller (COMPLETED)
*Objective:* Create reliable motor control with collision avoidance using STM32F401RC.

*Completed Tasks:*
- ✅ STM32 development environment setup
- ✅ MX1508 motor driver integration with PWM control
- ✅ UART communication interface (9600 baud)
- ✅ HC-SR04 ultrasonic sensors (Left, Right, Front)
- ✅ Real-time collision avoidance system
- ✅ FreeRTOS-based multitasking architecture
- ✅ Safety systems with emergency stops
- ✅ Status LED indicators

*Deliverable:* ✅ STM32 motor controller with 3-sensor collision avoidance

---

## ✅ Phase 2A — ToF Sensor Integration (COMPLETED)
*Objective:* Add precision ToF sensors for enhanced obstacle detection and junction recognition.

*Completed Tasks:*
- ✅ VL53L0X ToF sensor driver implementation
- ✅ 4-sensor array configuration (Front-Left, Front-Right, Left, Right)
- ✅ I2C1 bus setup on PB9/PB10 (avoiding pin conflicts)
- ✅ XSHUT control for address programming (PC0-PC3)
- ✅ ToF FreeRTOS task with 30ms measurement cycles
- ✅ Junction detection algorithms
- ✅ Obstacle detection with precise distance measurement
- ✅ UART command interface (T/J/O commands)
- ✅ Integration with existing collision avoidance system

*Deliverable:* ✅ 7-sensor navigation system (3 ultrasonic + 4 ToF)

---

## 🔄 Phase 2B — Raspberry Pi Integration (CURRENT PHASE)
*Objective:* Establish communication between Raspberry Pi and STM32 for vision-guided navigation.

*Tasks:*
- [ ] Set up Raspberry Pi 4B development environment
- [ ] Connect Pi UART to STM32 (existing protocol: 9600 baud)
- [ ] Test basic navigation commands from Pi to STM32:
  - Forward / Reverse / Left / Right / Stop
  - Speed control (1/2/3 commands)
  - Status queries (Q/I/U commands)
- [ ] Implement Python control interface on Raspberry Pi
- [ ] Test command reliability with collision avoidance active

*Deliverable:* Pi can control robot movements with collision safety

---

## 🧠 Phase 3 — Computer Vision Setup
*Objective:* Implement YOLO object detection and MobileNetV2 junction classification.

*Tasks:*
- [ ] Install Pi Camera or USB camera on Raspberry Pi
- [ ] Set up Python environment with OpenCV, PyTorch/TensorFlow
- [ ] Implement YOLO object detection for:
  - General obstacle detection (people, vehicles, etc.)
  - Special junction signs detection
- [ ] Implement MobileNetV2 junction classification:
  - Left turn junction
  - Right turn junction  
  - Straight path junction
  - Stop/End junction
- [ ] Create vision processing pipeline
- [ ] Test object detection accuracy in target environment

*Deliverable:* Vision system can detect obstacles and classify junctions

---

## 🎯 Phase 4 — Vision-Guided Navigation Logic
*Objective:* Integrate vision processing with navigation decisions.

*Tasks:*
- [ ] Implement navigation state machine:
  - EXPLORING (YOLO scanning for junction signs)
  - JUNCTION_DETECTED (MobileNetV2 classification)
  - NAVIGATING (following classification decision)
  - OBSTACLE_AVOIDANCE (temporary detour)
  - EMERGENCY_STOP (critical obstacle)
- [ ] Create decision logic:
  - Continuous YOLO monitoring for obstacles and signs
  - Trigger MobileNetV2 when junction sign detected
  - Convert classification to navigation commands
  - Handle obstacle avoidance priorities
- [ ] Implement command arbitration system
- [ ] Test vision-guided navigation in controlled environment

*Deliverable:* Robot autonomously navigates using vision guidance

---

## 🛡️ Phase 5 — Safety and Robustness
*Objective:* Enhance system reliability and safety features.

*Tasks:*
- [ ] Implement multi-layer safety system:
  - Hardware: STM32 collision avoidance (immediate)
  - Software: Vision obstacle detection (predictive)  
  - Failsafe: Emergency stop protocols
- [ ] Add error handling and recovery:
  - Camera failure detection
  - UART communication errors
  - Vision processing timeouts
- [ ] Implement performance monitoring:
  - Frame rate monitoring
  - Processing latency tracking
  - Memory usage optimization
- [ ] Stress testing under various conditions

*Deliverable:* Robust system with comprehensive safety features

---

## 📊 Phase 6 — Performance Optimization
*Objective:* Optimize system for real-time performance.

*Tasks:*
- [ ] Optimize computer vision pipeline:
  - Reduce YOLO model size for faster inference
  - Implement region of interest (ROI) processing
  - Multi-threading for vision processing
- [ ] Optimize communication protocol:
  - Implement message buffering
  - Add command prioritization
  - Reduce latency between vision and motion
- [ ] Battery and power optimization:
  - Monitor power consumption
  - Implement power-saving modes
  - Optimize processing duty cycles

*Deliverable:* Optimized system with real-time performance

---

## 🎮 Phase 7 — User Interface and Monitoring
*Objective:* Create monitoring and control interface.

*Tasks:*
- [ ] Develop web-based monitoring interface:
  - Real-time camera feed display
  - Navigation status dashboard
  - Sensor readings visualization
  - Manual override controls
- [ ] Implement remote monitoring:
  - Wi-Fi/network connectivity
  - Status reporting and logging
  - Performance metrics dashboard
- [ ] Create mobile app interface (optional)
- [ ] Add data logging and analysis tools

*Deliverable:* User-friendly monitoring and control interface

---

## 🧪 Phase 8 — Testing and Validation
*Objective:* Comprehensive testing and validation of complete system.

*Tasks:*
- [ ] Unit testing of all components
- [ ] Integration testing of vision + navigation
- [ ] Performance benchmarking
- [ ] Field testing in target environment
- [ ] Edge case testing and handling
- [ ] User acceptance testing
- [ ] Documentation and user guides

*Deliverable:* Fully tested and validated autonomous navigation system

---

# CURRENT PROJECT STATUS

## ✅ Completed Components
- **STM32F401RC Motor Controller**: Complete with 3-sensor collision avoidance
- **VL53L0X ToF Sensors**: 4-sensor array for precision obstacle detection and junction recognition
- **Hardware Setup**: MX1508 motor driver, HC-SR04 sensors, ToF sensors, UART communication
- **Safety Systems**: Multi-layer collision avoidance with emergency stops
- **Real-time Architecture**: FreeRTOS-based multitasking system

## 🔄 Current Focus: Hardware Assembly and Testing
**Next Immediate Tasks:**
1. **Physical ToF sensor mounting**: Mount 4x VL53L0X sensors on robot chassis
   - Front-Left and Front-Right sensors at 30° angles
   - Left and Right sensors for lateral wall detection  
2. **Wiring verification**: Connect I2C bus and XSHUT control pins
3. **System testing**: Verify ToF sensor functionality with test commands
4. **Raspberry Pi setup**: Prepare Pi 4B with camera and environment
5. **Communication validation**: Test enhanced sensor commands (T/J/O)

## 🎯 System Architecture Overview
```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS NAVIGATION SYSTEM                │
├─────────────────────┬───────────────────────────────────────────┤
│   RASPBERRY PI 4B   │              STM32F401RC                  │
│   (Vision & AI)     │           (Motor Controller)              │
├─────────────────────┼───────────────────────────────────────────┤
│ 🧠 YOLO Object      │ ⚙️  Motor Control (MX1508)               │
│    Detection        │ 📡 UART Communication (9600 baud)        │
│ 🔍 MobileNetV2      │ 🛡️  Collision Avoidance (HC-SR04 x3)     │
│    Junction Class.  │ 🎯 Precision Distance (VL53L0X x4)       │
│ 📷 Camera Feed      │ 🔧 Real-time Safety Systems              │
│    Processing       │ 💾 FreeRTOS Architecture                 │
│                     │ 📊 Status Reporting (I2C + UART)        │
└─────────────────────┴───────────────────────────────────────────┘

Sensor Array Configuration:
┌─────────────────────────────────────────────────────────────────┐
│                         ROBOT TOP VIEW                         │
│                                                                 │
│  ToF_FL   [🤖]    ToF_FR      HC-SR04 Ultrasonic (Backup)      │
│   (30°)   FRONT    (30°)      • Left Wall Detection (A)        │
│     \       |       /         • Right Wall Detection (B)       │
│      \      |      /          • Front Collision (C)            │
│       \     |     /                                             │
│ ToF_L ──────────────── ToF_R  VL53L0X ToF (Primary)            │
│ (90°)               (90°)     • Front-Left Junction (FL)        │
│                               • Front-Right Junction (FR)       │
│                               • Left Wall Following (L)         │
│                               • Right Wall Following (R)        │
└─────────────────────────────────────────────────────────────────┘
```

# UPDATED PROJECT TO-DO LIST

## ✅ Phase 1: STM32 Motor Controller (COMPLETED)
- [x] STM32F401RC development environment setup  
- [x] MX1508 motor driver integration with PWM
- [x] UART communication protocol (9600 baud)
- [x] HC-SR04 ultrasonic sensors (Left, Right, Front)
- [x] Real-time collision avoidance algorithms
- [x] FreeRTOS multitasking architecture  
- [x] Safety systems with emergency stops
- [x] Status LED indicators and debugging
- [x] Complete modular code architecture

## ✅ Phase 2A: ToF Sensor Integration (COMPLETED) 
- [x] VL53L0X ToF sensor driver implementation
- [x] I2C1 bus configuration (PB9/PB10)
- [x] XSHUT address programming (PC0-PC3)
- [x] 4-sensor array setup (FL, FR, L, R)
- [x] ToF FreeRTOS task with 30ms cycles
- [x] Junction detection algorithms
- [x] Enhanced obstacle detection
- [x] UART command interface (T/J/O commands)
- [x] Pin conflict avoidance and integration

## 🔄 Phase 2B: Raspberry Pi Integration (CURRENT)
- [ ] Set up Raspberry Pi 4B development environment
- [ ] Install Python libraries (OpenCV, PyTorch/TensorFlow, pySerial)
- [ ] Connect Pi UART to STM32 (GPIO 14/15 to PA10/PA9)
- [ ] Test basic UART communication
- [ ] Implement Python motor control interface
- [ ] Test navigation commands: F/R/L/T/S/1/2/3
- [ ] Test sensor query commands: Q/I/U  
- [ ] Validate collision avoidance during Pi control
- [ ] Create robust communication error handling

## 🧠 Phase 3: Computer Vision Setup
- [ ] Install Pi Camera Module or USB camera
- [ ] Set up camera capture and preview
- [ ] Install YOLO object detection model
- [ ] Train/configure MobileNetV2 for junction signs
- [ ] Test object detection performance
- [ ] Optimize models for Raspberry Pi performance
- [ ] Implement vision processing pipeline
- [ ] Test detection accuracy in target environment
- [ ] Create vision debugging and monitoring tools

## 🎯 Phase 4: Vision-Guided Navigation
- [ ] Design navigation state machine
- [ ] Implement YOLO continuous monitoring
- [ ] Create junction sign detection triggers
- [ ] Implement MobileNetV2 classification logic  
- [ ] Convert vision decisions to STM32 commands
- [ ] Create command arbitration system
- [ ] Test vision-guided navigation
- [ ] Handle edge cases and error conditions
- [ ] Optimize decision-making algorithms

## 🛡️ Phase 5: Safety and Robustness
- [ ] Multi-layer safety system implementation
- [ ] Vision processing timeout handling
- [ ] Camera failure detection and recovery
- [ ] UART communication error recovery
- [ ] Emergency stop protocol enhancement
- [ ] Performance monitoring implementation
- [ ] Memory usage optimization
- [ ] Stress testing under various conditions
- [ ] Backup navigation modes

## 📊 Phase 6: Performance Optimization  
- [ ] Vision pipeline optimization (ROI, threading)
- [ ] YOLO model size optimization for Pi
- [ ] Real-time processing optimization
- [ ] Communication latency reduction
- [ ] Power consumption optimization
- [ ] Frame rate optimization
- [ ] Memory leak prevention
- [ ] CPU usage optimization

## 🎮 Phase 7: Monitoring Interface
- [ ] Web-based dashboard development
- [ ] Real-time camera feed display
- [ ] Navigation status visualization
- [ ] Sensor data monitoring  
- [ ] Manual override controls
- [ ] Performance metrics dashboard
- [ ] Data logging implementation
- [ ] Mobile-responsive interface

## 🧪 Phase 8: Testing and Validation
- [ ] Unit testing of all components
- [ ] Vision system accuracy testing
- [ ] Navigation reliability testing
- [ ] Safety system validation
- [ ] Performance benchmarking
- [ ] Field testing in target environment
- [ ] Edge case testing
- [ ] User acceptance testing
- [ ] Documentation completion

---

## Key Technology Stack
- **STM32F401RC**: Motor control, collision avoidance, real-time safety
- **Raspberry Pi 4B**: Computer vision, AI processing, high-level navigation
- **YOLO**: General object detection and junction sign detection
- **MobileNetV2**: Junction classification (left/right/straight)
- **HC-SR04**: Ultrasonic collision avoidance (6ms response time)
- **FreeRTOS**: Real-time multitasking on STM32
- **Python**: High-level control and vision processing

**Total Tasks:** 64  
**Phases:** 8  
**Current Status:** Phase 1 Complete, Phase 2 Starting

---

*Last Updated: November 16, 2025*

