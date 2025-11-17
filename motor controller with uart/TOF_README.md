# ToF Sensors Integration - VL53L0X Module

## Overview
The ToF (Time of Flight) sensor module provides precision distance measurement using 4 VL53L0X sensors for enhanced obstacle detection and junction recognition. This system works alongside the existing HC-SR04 ultrasonic sensors to create a robust 7-sensor navigation array.

## Hardware Configuration

### Pin Assignments
**I2C1 Bus (Shared by all 4 sensors):**
- `PB9` → I2C1_SDA (Serial Data Line)
- `PB10` → I2C1_SCL (Serial Clock Line)

**XSHUT Control Pins (Address Programming):**
- `PC0` → ToF_FL_XSHUT (Front-Left sensor shutdown)
- `PC1` → ToF_FR_XSHUT (Front-Right sensor shutdown)
- `PC2` → ToF_L_XSHUT (Left sensor shutdown)
- `PC3` → ToF_R_XSHUT (Right sensor shutdown)

### Sensor Placement
```
        Front-Left    Front-Right
             \           /
              \   [🤖]   /
               \  ROBOT /
                \      /
     Left ────────────────── Right
     (60°)                  (60°)
```

- **Front-Left (FL)**: 30° angle coverage for junction detection
- **Front-Right (FR)**: 30° angle coverage for junction detection
- **Left (L)**: 60° lateral coverage for wall following
- **Right (R)**: 60° lateral coverage for wall following

### I2C Addresses
After initialization, sensors are programmed with unique addresses:
- `0x54` → Front-Left (ToF_FL)
- `0x56` → Front-Right (ToF_FR)
- `0x58` → Left (ToF_L)
- `0x5A` → Right (ToF_R)

## Software Architecture

### Key Files
- `Core/Inc/tof_sensors.h` - Header file with API definitions
- `Core/Src/tof_sensors.c` - Implementation with VL53L0X driver
- Integration with existing command processor and main loop

### FreeRTOS Task
- **Task Name**: `tofTask`
- **Priority**: `osPriorityNormal`
- **Stack Size**: 512 words
- **Cycle Time**: 30ms per sensor (120ms full cycle)

### API Functions

#### Initialization
```c
HAL_StatusTypeDef ToF_Init(void);                    // Complete system init
HAL_StatusTypeDef ToF_GPIO_Init(void);               // GPIO configuration
HAL_StatusTypeDef ToF_I2C_Init(void);                // I2C bus setup
HAL_StatusTypeDef ToF_Sensors_Init(void);            // Sensor address programming
```

#### Data Access
```c
uint16_t ToF_GetDistance(ToF_Sensor_ID_t sensor);    // Get distance (mm)
bool ToF_IsValid(ToF_Sensor_ID_t sensor);            // Check data validity
uint32_t ToF_GetAge(ToF_Sensor_ID_t sensor);         // Get data age (ms)
```

#### Detection Functions
```c
bool ToF_DetectJunction(void);                       // Full junction detection
bool ToF_DetectLeftJunction(void);                   // Left opening detection
bool ToF_DetectRightJunction(void);                  // Right opening detection
bool ToF_DetectFrontObstacle(void);                  // Front obstacle detection
```

#### Navigation Support
```c
uint16_t ToF_GetLeftWallDistance(void);              // Left wall distance
uint16_t ToF_GetRightWallDistance(void);             // Right wall distance
int16_t ToF_GetCenteringError(void);                 // Centering calculation
```

## UART Commands

### New Commands Added
- **`G`** - Get ToF sensor distances (changed from T to avoid conflict)
- **`J`** - Get junction detection status
- **`O`** - Get obstacle detection status

### Command Examples

#### Distance Query (`G` command)
```
Send: G
Response: ToF: FL=245mm FR=312mm L=156mm R=189mm
```

#### Junction Detection (`J` command)
```
Send: J
Response: Junction: L=1 R=0 Both=0
```

#### Obstacle Detection (`O` command)
```
Send: O
Response: Obstacle: Front=0 Closest=156mm
```

## Detection Thresholds

### Junction Detection
- **Threshold**: 300mm (30cm)
- **Logic**: Distance > 300mm indicates open path (junction)

### Obstacle Detection
- **Stop Threshold**: 150mm (15cm) - Emergency stop
- **Slow Threshold**: 300mm (30cm) - Reduce speed
- **Wall Following**: 200mm (20cm) - Optimal wall distance

### Measurement Specifications
- **Range**: 30mm - 2000mm (3cm - 2m)
- **Accuracy**: ±3% typical
- **Update Rate**: 30ms per sensor
- **Angular Coverage**: 25° cone (typical)

## Integration with Existing Systems

### Collision Avoidance Hierarchy
1. **Primary**: HC-SR04 ultrasonic sensors (immediate collision prevention)
2. **Secondary**: VL53L0X ToF sensors (precise obstacle detection)
3. **Tertiary**: Vision system integration (future Phase 3)

### Safety Features
- Sensor validity checking before using data
- Timeout handling for failed measurements
- Graceful degradation if sensors fail
- Hardware XSHUT for sensor reset capability

## Troubleshooting

### Common Issues

#### 1. Sensor Not Detected
**Symptoms**: Boot message shows sensor not found
**Solutions**:
- Verify I2C wiring (SDA/SCL connections)
- Check XSHUT pin connections
- Ensure 3.3V power supply to sensors
- Verify pull-up resistors on I2C lines

#### 2. Address Programming Failed
**Symptoms**: Multiple sensors respond at same address
**Solutions**:
- Check XSHUT control pins (PC0-PC3)
- Verify sensor power-up sequence
- Check for I2C bus conflicts

#### 3. Invalid Distance Readings
**Symptoms**: ToF command shows invalid data
**Solutions**:
- Check sensor mounting and alignment
- Verify measurement environment (avoid reflective surfaces)
- Check for I2C communication errors
- Increase measurement timeout if needed

### Debug Commands
```c
ToF_SelfTest();        // Complete sensor test
ToF_PrintStatus();     // Detailed status report
```

## Performance Characteristics

### Timing Analysis
- **Sensor Init Time**: ~100ms (all 4 sensors)
- **Single Measurement**: ~30ms
- **Full Cycle**: 120ms (4 sensors)
- **I2C Speed**: 100kHz (standard mode)

### Memory Usage
- **RAM**: ~200 bytes for sensor data
- **Stack**: 512 words for ToF task
- **Flash**: ~8KB for driver code

## Future Enhancements

### Planned Improvements
1. **Adaptive Measurement**: Adjust cycles based on navigation mode
2. **ROI Processing**: Focus on specific areas during turns
3. **Sensor Fusion**: Combine with ultrasonic data for better accuracy
4. **Calibration**: Environment-specific threshold tuning
5. **Error Recovery**: Automatic sensor reset on failures

### Vision System Integration
- Junction classification validation
- Obstacle size estimation
- Path planning support
- Dynamic threshold adjustment

## Wiring Diagram

```
VL53L0X Sensor Connections (All 4 sensors):
┌─────────────┬─────────────┐
│ VL53L0X Pin │ STM32 Pin   │
├─────────────┼─────────────┤
│ VDD         │ 3.3V        │
│ GND         │ GND         │
│ SCL         │ PB10 (I2C1) │
│ SDA         │ PB9 (I2C1)  │
│ XSHUT       │ PC0-PC3     │
│ GPIO1       │ Not Used    │
└─────────────┴─────────────┘

XSHUT Control Assignments:
• ToF_FL (Front-Left): PC0
• ToF_FR (Front-Right): PC1  
• ToF_L (Left): PC2
• ToF_R (Right): PC3
```

---

*Last Updated: November 16, 2025*  
*Integration Status: ✅ Complete - Ready for hardware assembly and testing*