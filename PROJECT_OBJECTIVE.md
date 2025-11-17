# 🤖 Autonomous Robot Project - STM32 Motor Controller

## 🎯 **PROJECT OBJECTIVE**

### **System Overview**
This project implements a **distributed autonomous navigation system** with computer vision capabilities:

```
┌─────────────────────────────────────────────────────────────────┐
│                    AUTONOMOUS ROBOT SYSTEM                     │
├─────────────────────┬───────────────────────────────────────────┤
│   RASPBERRY PI 4B   │              STM32F401RC                  │
│   (Vision & AI)     │           (Motor Controller)              │
├─────────────────────┼───────────────────────────────────────────┤
│                     │                                           │
│ 🧠 YOLO Object      │ ⚙️  Motor Control (MX1508 H-Bridge)       │
│    Detection        │     • Differential drive                  │
│                     │     • PWM speed control                   │
│ 🔍 MobileNetV2      │     • Smooth acceleration                 │
│    Junction         │                                           │
│    Classification   │ 📡 UART Communication                     │
│                     │     • Command reception                   │
│ 📷 Camera Feed      │     • Status reporting                    │
│    Processing       │     • Real-time feedback                  │
│                     │                                           │
│ 🎯 Navigation       │ 🛡️  Safety Systems                        │
│    Decisions        │     • Collision avoidance                 │
│                     │     • Emergency stops                     │
│                     │     • Timeout protection                  │
│                     │                                           │
│         UART        │ 📊 Sensor Integration                     │
│      Commands       │     • 3x HC-SR04 Ultrasonic               │
│         ↓           │     • Left/Right wall detection           │
│                     │     • Front obstacle detection            │
└─────────────────────┴───────────────────────────────────────────┘
```

---

## 🎮 **OPERATIONAL WORKFLOW**

### **Phase 1: Computer Vision Processing (Raspberry Pi)**
1. **YOLO Object Detection** continuously analyzes camera feed
2. **When special junction sign detected** → Triggers MobileNetV2
3. **MobileNetV2** classifies junction type (left/right/straight)
4. **Navigation decision** made based on classification
5. **UART commands sent** to STM32 for motor control

### **Phase 2: Motor Control & Safety (STM32)**
1. **Receive navigation commands** via UART from Raspberry Pi
2. **Execute motor movements** with PWM control
3. **Continuous collision monitoring** using 3 ultrasonic sensors
4. **Emergency stop** if obstacles detected
5. **Status feedback** to Raspberry Pi

---

## 🛠️ **HARDWARE ARCHITECTURE**

### **STM32F401RC Motor Controller**
```c
Hardware Connections:
├── Motor Control (MX1508)
│   ├── PA0 → Motor1 Forward  (TIM5_CH1 PWM)
│   ├── PA1 → Motor1 Reverse  (TIM5_CH2 PWM)
│   ├── PA2 → Motor2 Forward  (TIM5_CH3 PWM)
│   └── PA3 → Motor2 Reverse  (TIM5_CH4 PWM)
│
├── UART Communication
│   ├── PA9  → TX (to Raspberry Pi)
│   └── PA10 → RX (from Raspberry Pi)
│
├── Ultrasonic Sensors (HC-SR04)
│   ├── PB0 → Left Trigger,    PB6 → Left Echo
│   ├── PB1 → Right Trigger,   PB7 → Right Echo
│   └── PB2 → Front Trigger,   PB8 → Front Echo
│
└── Status LEDs
    ├── PC13 → RX Activity
    ├── PC14 → TX Activity  
    ├── PB12 → Heartbeat
    ├── PB14 → Left Wall Detection
    └── PB15 → Right Wall Detection
```

---

## 📡 **COMMUNICATION PROTOCOL**

### **Raspberry Pi → STM32 Commands**
```c
Navigation Commands:
'F' → Move Forward
'R' → Move Reverse
'L' → Turn Left (spot turn)
'T' → Turn Right (spot turn)
'S' → Stop

Speed Control:
'1' → Slow speed (40%)
'2' → Medium speed (70%)
'3' → Fast speed (100%)

Motion Control:
'M' → Enable smooth acceleration
'Z' → Disable smooth acceleration
'E' → Emergency stop

Sensor Queries:
'Q' → Query front distance
'I' → Full status report
'U' → Test all sensors
```

### **STM32 → Raspberry Pi Responses**
```c
Status Reports:
"FD=25\r\n"                    // Front distance = 25cm
"STATUS L=12 R=15 F=25 SPD=70 MOV=Y\r\n"  // Full status
"US L=12cm R=15cm F=25cm\r\n"  // Sensor test result

Error Conditions:
"COLLISION\r\n"                // Emergency stop triggered
"TIMEOUT\r\n"                  // No commands received
"ERROR\r\n"                    // Hardware fault
```

---

## 🧠 **SOFTWARE ARCHITECTURE (STM32)**

### **RTOS Task Structure**
```c
┌─ defaultTask (Normal Priority)     ─┐
│  • System health monitoring         │
│  • Heartbeat LED                    │
│  • Debug output                     │
└─────────────────────────────────────┘

┌─ motorTask (Above Normal Priority) ─┐
│  • UART command processing          │
│  • Motor control execution          │
│  • Safety timeout monitoring        │
└─────────────────────────────────────┘

┌─ ultrasonicTask (Normal Priority)  ─┐
│  • Wall avoidance algorithm         │
│  • Front obstacle detection         │
│  • Collision prevention             │
│  • PID center-seeking control       │
└─────────────────────────────────────┘
```

### **Core Modules**
- **`motor_control.c`**: PWM motor control with smooth acceleration
- **`uart_comm.c`**: UART communication and command queuing
- **`ultrasonic.c`**: HC-SR04 distance measurement
- **`command_processor.c`**: Command parsing and execution
- **`wall_avoidance.c`**: Collision avoidance algorithms
- **`led_indicators.c`**: Status indication system

---

## 🛡️ **SAFETY SYSTEMS**

### **Multi-Layer Safety Architecture**
```c
Safety Layer 1: Hardware
├── PWM motor control (no sudden jerks)
├── GPIO-based emergency stops
└── Watchdog timer protection

Safety Layer 2: Software
├── 10-second command timeout → auto-stop
├── UART error detection → immediate stop  
├── Queue overflow protection
└── Stack overflow monitoring

Safety Layer 3: Collision Avoidance
├── Front: 15cm stop, 30cm slow, 50cm warn
├── Sides: 2.5cm stop, 5cm correct
├── 20Hz real-time monitoring
└── Emergency brake capability
```

---

## 🚀 **INTEGRATION WORKFLOW**

### **Development Phases**
1. **✅ STM32 Motor Controller** (Current - Complete)
   - Motor control with PWM
   - UART communication
   - 3-sensor collision avoidance
   - RTOS-based real-time operation

2. **🔄 Raspberry Pi Integration** (Next Phase)
   - UART communication setup
   - Command protocol implementation
   - Basic navigation testing

3. **🧠 Computer Vision Integration** (Future)
   - YOLO object detection setup
   - MobileNetV2 junction classification
   - Camera feed processing
   - Decision-making algorithms

4. **🎯 Full Autonomous Operation** (Final)
   - End-to-end testing
   - Performance optimization
   - Edge case handling
   - Field deployment

---

## 📊 **PERFORMANCE SPECIFICATIONS**

### **Real-Time Performance**
- **Motor Response**: < 1ms command to action
- **Collision Detection**: 20Hz update rate (50ms intervals)
- **UART Communication**: 9600 baud with interrupt handling
- **Safety Timeout**: 10-second emergency stop
- **Sensor Range**: HC-SR04: 2cm - 400cm, 6ms response time

### **Power & Efficiency**
- **STM32F401RC**: 16MHz HSI, low power consumption
- **Motor Control**: 1kHz PWM, efficient H-bridge operation
- **Sensor Power**: Minimal power HC-SR04 operation

---

## 🔧 **DEVELOPER NOTES**

### **Key Design Decisions**
1. **HC-SR04 over VL53L0X**: 6ms vs 40ms response time for collision avoidance
2. **FreeRTOS**: Real-time operation with proper task priorities
3. **Modular Architecture**: Easy to maintain and extend
4. **Hardware PWM**: Smooth motor control without CPU overhead
5. **Interrupt-driven UART**: Non-blocking command processing

### **Extension Points**
- Add more sensors (IMU, encoders, GPS)
- Implement advanced navigation algorithms
- Add wireless communication (WiFi, Bluetooth)
- Integrate additional safety features
- Expand command set for complex maneuvers

---

## 📚 **FILE STRUCTURE**
```
motor controller with uart/
├── Core/
│   ├── Inc/
│   │   ├── main.h                  // Hardware definitions
│   │   ├── motor_control.h         // Motor control API
│   │   ├── uart_comm.h            // UART communication
│   │   ├── ultrasonic.h           // Sensor interface
│   │   ├── command_processor.h    // Command handling
│   │   ├── wall_avoidance.h       // Safety algorithms
│   │   └── led_indicators.h       // Status display
│   └── Src/
│       ├── main.c                 // System initialization
│       ├── motor_control.c        // PWM motor control
│       ├── uart_comm.c           // Communication layer
│       ├── ultrasonic.c          // Sensor implementation
│       ├── command_processor.c   // Command parsing
│       ├── wall_avoidance.c      // Safety implementation
│       └── led_indicators.c      // LED control
├── Debug/                         // Build artifacts
├── Drivers/                       // HAL drivers
└── PROJECT_OBJECTIVE.md          // This file
```

---

## 🎯 **QUICK START FOR AI ASSISTANTS**

When working with this codebase:
1. **This is a motor controller** - not the vision system
2. **Commands come from Raspberry Pi** via UART
3. **Safety is paramount** - collision avoidance is critical
4. **Real-time operation** - don't block the main tasks
5. **Modular design** - changes should respect module boundaries
6. **HC-SR04 sensors** are the collision detection system

**Current Status**: STM32 motor controller is complete and functional. Ready for Raspberry Pi integration.
