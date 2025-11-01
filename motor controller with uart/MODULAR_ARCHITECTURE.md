# Modular Architecture Documentation

## Overview

The motor controller firmware has been reorganized into a clean, modular architecture for better maintainability, readability, and scalability. The monolithic `main.c` file (1200+ lines) has been split into 6 specialized modules.

## File Structure

```
Core/
├── Inc/
│   ├── main.h                  # Main header with hardware definitions
│   ├── motor_control.h         # Motor control API
│   ├── uart_comm.h             # UART communication API
│   ├── led_indicators.h        # LED control API
│   ├── ultrasonic.h            # Ultrasonic sensor API
│   ├── command_processor.h     # Command processing API
│   └── wall_avoidance.h        # Wall following/avoidance API
│
└── Src/
    ├── main.c                  # Main program (initialization + RTOS tasks)
    ├── main_backup.c           # Original monolithic file (backup)
    ├── motor_control.c         # Motor control implementation
    ├── uart_comm.c             # UART communication implementation
    ├── led_indicators.c        # LED control implementation
    ├── ultrasonic.c            # Ultrasonic sensor implementation
    ├── command_processor.c     # Command processing implementation
    └── wall_avoidance.c        # Wall following/avoidance implementation
```

## Module Breakdown

### 1. **motor_control.c/h** - Motor Control Module
**Purpose:** PWM-based motor control with smooth acceleration/deceleration

**Functions:**
- `Motor_GPIO_Init()` - Configure motor GPIO pins
- `Motor_TIM5_PWM_Init()` - Initialize PWM timer
- `Motor_SetSpeed()` - Set motor speeds instantly
- `Motor_SetSpeed_Smooth()` - Set speeds with ramping
- `Motor_Forward()` - Move both motors forward
- `Motor_Reverse()` - Move both motors backward
- `Motor_Left()` - Spot turn left
- `Motor_Right()` - Spot turn right
- `Motor_ForwardDifferential()` - Arc steering with different wheel speeds
- `Motor_Stop()` - Instant stop
- `Motor_Stop_Smooth()` - Gradual deceleration stop

**Exported Variables:**
- `current_speed` - Current speed setting (SLOW/MEDIUM/FAST)
- `accel_enabled` - Smooth ramping enable flag
- `current_m1_in1/2` - Current PWM values for Motor 1
- `current_m2_in3/4` - Current PWM values for Motor 2
- `motors_moving` - Movement state flag
- `last_movement_cmd` - Last direction command

---

### 2. **uart_comm.c/h** - UART Communication Module
**Purpose:** Serial communication and debug output

**Functions:**
- `UART_GPIO_Init()` - Configure UART GPIO pins
- `UART_Init()` - Initialize USART1 peripheral (9600 baud)
- `UART_IRQHandler()` - UART interrupt handler
- `UART_SendString()` - Send string (polling mode)
- `UART_SendUInt()` - Send unsigned integer
- `UART_SendCRLF()` - Send carriage return + line feed

**Exported Variables:**
- `rx_data` - Last received byte
- `uart_error` - Error flag
- `uart_rx_seen` - First RX flag (stops heartbeat spam)
- `uartCmdQueueHandle` - FreeRTOS message queue handle

**Interrupt Handler:**
- Processes RXNE, TXE, ORE, FE, NE, PE flags
- Pushes received bytes to queue
- Toggles RX/TX LEDs

---

### 3. **led_indicators.c/h** - LED Indicators Module
**Purpose:** Visual feedback for system status

**Functions:**
- `LED_Init()` - Configure all LED GPIO pins
- `LED_Toggle_RX()` - Toggle RX activity LED
- `LED_Toggle_TX()` - Toggle TX activity LED
- `LED_Toggle_Heartbeat()` - Toggle heartbeat LED
- `LED_SetWallIndicators()` - Set wall detection LED states
- `LED_SelfTest()` - Brief blink sequence at boot

**LEDs:**
- **PC13** - RX activity (toggles on data reception)
- **PC14** - TX activity (toggles on data transmission)
- **PB12** - Heartbeat (blinks at 4Hz)
- **PB14** - Left wall proximity warning
- **PB15** - Right wall proximity warning

---

### 4. **ultrasonic.c/h** - Ultrasonic Sensor Module
**Purpose:** HC-SR04 distance measurement and collision detection

**Functions:**
- `Ultrasonic_GPIO_Init()` - Configure ultrasonic GPIO pins
- `Ultrasonic_Init()` - Initialize DWT cycle counter for μs timing
- `Ultrasonic_MeasureA()` - Measure left sensor distance (0-400cm)
- `Ultrasonic_MeasureB()` - Measure right sensor distance (0-400cm)
- `Ultrasonic_CheckCollision()` - Check if collision imminent

**Private Functions:**
- `DWT_Delay_Init()` - Enable ARM DWT cycle counter
- `micros()` - Get microseconds since startup
- `delay_us()` - Busy-wait microsecond delay
- `Ultrasonic_Measure_Pin()` - Core measurement logic

**Exported Variables:**
- `ultrasonic_left_cm` - Left sensor reading
- `ultrasonic_right_cm` - Right sensor reading

**Hardware:**
- **PB0, PB1** - Trigger pins (output)
- **PB6, PB7** - Echo pins (input with pulldown)

---

### 5. **command_processor.c/h** - Command Processing Module
**Purpose:** Parse and execute UART commands with safety checks

**Functions:**
- `Command_Process()` - Process received UART command
- `Command_SafetyCheck()` - Emergency stop if no command for 10s

**Supported Commands:**
- **F** - Forward
- **R** - Reverse
- **L** - Left (spot turn)
- **T** - Right (spot turn)
- **S** - Stop
- **E** - Emergency stop
- **1** - Speed SLOW (40%)
- **2** - Speed MEDIUM (70%)
- **3** - Speed FAST (100%)
- **M** - Enable smooth acceleration
- **Z/D** - Disable smooth acceleration
- **X** - Self-test sequence
- **U** - Ultrasonic ping (measure and report)

**Exported Variables:**
- `last_command_time` - Timestamp for timeout detection

**Safety Features:**
- 10-second command timeout (configurable via `SAFETY_TIMEOUT_MS`)
- Dynamic speed changes while moving
- Smooth vs instant stop based on acceleration setting

---

### 6. **wall_avoidance.c/h** - Wall Avoidance Module
**Purpose:** PID-based wall following and collision avoidance

**Functions:**
- `WallAvoidance_Task()` - FreeRTOS task (20Hz measurement loop)

**Features:**
- **Emergency Stop:** Stops motors if either sensor < 2.5cm
- **PID Center-Seeking:** Steers to stay centered between walls
  - Kp = 1.0 (proportional correction)
  - Ki = 0.0 (integral disabled to avoid windup)
  - Kd = 0.2 (derivative damping)
  - Deadband = 1.0cm (avoid oscillation)
  - Max correction = ±40%
- **Fallback Proportional Control:** Single-sensor nudge if only one valid
- **Visual Feedback:** Wall LEDs light when within slow zone (5cm)
- **Debug Output:** Prints distances, LED states, command every ~500ms

**PID Algorithm:**
```c
error = left_distance - right_distance  // positive = closer to left
correction = Kp*error + Ki*integral + Kd*derivative
left_motor = base_speed - correction
right_motor = base_speed + correction
```

**Collision Thresholds:**
- **STOP:** 2.5cm (hard emergency stop)
- **SLOW:** 5cm (steering corrections applied)
- **WARN:** 50cm (future use)

---

### 7. **main.c** - Main Program
**Purpose:** System initialization and RTOS task definitions

**Content:**
- `main()` - Entry point, peripheral init, RTOS startup
- `SystemClock_Config()` - 16MHz HSI clock setup
- `StartDefaultTask()` - Idle/housekeeping task (heartbeat)
- `StartMotorTask()` - Command processing task (high priority)
- `USART1_IRQHandler()` - Redirects to `UART_IRQHandler()`

**Initialization Sequence:**
1. HAL_Init()
2. SystemClock_Config()
3. Motor_GPIO_Init()
4. UART_GPIO_Init()
5. LED_Init()
6. Ultrasonic_GPIO_Init() (if enabled)
7. Motor_TIM5_PWM_Init()
8. UART_Init()
9. Ultrasonic_Init() (if enabled)
10. LED_SelfTest()
11. Create RTOS queues and tasks
12. osKernelStart()

**RTOS Tasks:**
- **defaultTask** - Priority: Normal, Stack: 128 bytes
  - Heartbeat LED blinking
  - Optional UART heartbeat debug
- **motorTask** - Priority: AboveNormal, Stack: 256 bytes
  - Read commands from queue
  - Execute motor commands
  - Safety timeout enforcement
- **ultrasonicTask** - Priority: Normal, Stack: 256 bytes
  - Measure distances (20Hz)
  - Apply wall avoidance steering
  - Emergency stop on collision

---

## Benefits of Modular Design

### **Improved Maintainability**
- Each module has a single, well-defined responsibility
- Changes to motor control don't affect UART communication
- Easier to locate and fix bugs

### **Code Reusability**
- Modules can be reused in other projects
- Clear API boundaries make testing easier
- Can replace implementations without touching other modules

### **Scalability**
- Easy to add new features (e.g., IR sensors, encoders)
- Can create new modules without refactoring existing code
- Supports team collaboration (different developers per module)

### **Readability**
- Reduced cognitive load (200-300 lines per file vs 1200+)
- Clear naming conventions
- Comprehensive documentation

### **Compilation Efficiency**
- Only modified modules need recompilation
- Faster build times during development

---

## Build System Integration

### **STM32CubeIDE / Makefile**
All `.c` files in `Core/Src/` are automatically included in the build. No changes needed.

### **Keil MDK / IAR**
Add the following files to your project:
- `motor_control.c`
- `uart_comm.c`
- `led_indicators.c`
- `ultrasonic.c`
- `command_processor.c`
- `wall_avoidance.c`

Include paths should already contain `Core/Inc/`.

---

## Migration Notes

### **What Changed:**
1. **main.c** reduced from 1200+ lines to ~400 lines
2. All function implementations moved to specialized modules
3. Global variables now properly scoped with `extern` declarations
4. Interrupt handlers redirect to module functions

### **What Stayed the Same:**
- All hardware connections (no rewiring needed)
- Command protocol (same UART commands)
- Motor control behavior (same PWM, acceleration, etc.)
- RTOS task structure (same 3 tasks)
- Functionality is 100% identical

### **Backward Compatibility:**
- Original `main.c` backed up as `main_backup.c`
- Can revert by copying `main_backup.c` to `main.c`
- All configuration macros in `main.h` unchanged

---

## Testing Checklist

After compilation, verify:

- [ ] Firmware compiles without errors
- [ ] UART commands work (F, R, L, T, S, 1, 2, 3, M, Z, X, U)
- [ ] Motors respond correctly to commands
- [ ] LEDs blink on RX/TX activity
- [ ] Heartbeat LED blinks at 4Hz
- [ ] Wall LEDs light when sensors detect proximity
- [ ] Emergency stop triggers if no command for 10s
- [ ] Ultrasonic measurements print correctly (U command)
- [ ] PID steering works when moving forward near walls
- [ ] Boot banner prints on startup (if DEBUG_BOOT_BANNER=1)

---

## Troubleshooting

### **Linker Errors:**
- Ensure all `.c` files are in build system
- Check that `Core/Inc/` is in include paths
- Rebuild entire project (clean + build)

### **Missing Symbols:**
- Add `extern` declarations in headers
- Check spelling of function/variable names
- Ensure module headers are included in main.c

### **Runtime Issues:**
- Verify GPIO initialization order (GPIO before peripherals)
- Check stack sizes if tasks crash (increase if needed)
- Enable DEBUG_BOOT_BANNER to see boot sequence

---

## Future Enhancements

The modular design makes it easy to add:

1. **IR Sensor Module** (`ir_sensors.c/h`)
   - Analog distance measurement
   - Line following logic

2. **Encoder Module** (`encoders.c/h`)
   - Wheel rotation tracking
   - Odometry calculations

3. **Bluetooth Module** (`bluetooth.c/h`)
   - Wireless command interface
   - Telemetry streaming

4. **Path Planning Module** (`path_planning.c/h`)
   - Autonomous navigation
   - Maze solving algorithms

5. **Data Logging Module** (`data_logger.c/h`)
   - SD card logging
   - Performance metrics

---

## Author Notes

**Version:** 2.2 - Modular Architecture  
**Date:** November 1, 2025  
**Changes:** Complete refactoring into 6 specialized modules  

This refactoring maintains 100% functional compatibility while dramatically improving code organization and maintainability. All features from the original monolithic design are preserved.
