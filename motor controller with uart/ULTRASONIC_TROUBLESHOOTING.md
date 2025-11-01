# Ultrasonic Sensor Troubleshooting Guide

## Problem: Sensors Don't Respond (Both Show 0cm)

### Quick Diagnostic Steps

1. **Check if firmware ultrasonic is enabled**
   ```c
   // In Core/Inc/main.h, verify:
   #define ULTRASONIC_ENABLED 1
   #define ULTRASONIC_DEBUG 1
   ```

2. **Verify STM32 debug output**
   - Connect to UART (9600 baud)
   - You should see: `US A=XXcm B=YYcm` every 500ms
   - If nothing appears, ultrasonic task not running

3. **Run Python script and check for warnings**
   ```bash
   python3 rpi_motor_controller.py
   # Press 'U' to manually ping sensors
   # Script will warn: "WARNING: Both ultrasonic sensors reading 0cm"
   ```

---

## Hardware Troubleshooting

### Power Supply Issues

**Symptom**: Both sensors read 0cm constantly

**Causes & Solutions**:
- ✅ **HC-SR04 needs 5V**: Connect VCC to 5V rail (NOT 3.3V!)
- ✅ **Current draw**: Each sensor draws ~15mA, ensure supply can handle 30mA+
- ✅ **Add capacitors**: 100μF electrolytic near sensors to stabilize power

**How to test**:
```bash
# Measure voltage at sensor VCC pins with multimeter
# Should read 4.75V - 5.25V
```

---

### Wiring Verification Checklist

#### Left Sensor (A) - Pin PB0/PB6

| HC-SR04 Pin | Wire Color | Destination | Voltage |
|------------|-----------|-------------|---------|
| VCC | Red | 5V rail | 5.0V |
| TRIG | Purple | STM32 PB0 | 3.3V (output) |
| ECHO | Pink | STM32 PB6* | 5V (needs divider!) |
| GND | Black | Common GND | 0V |

#### Right Sensor (B) - Pin PB1/PB7

| HC-SR04 Pin | Wire Color | Destination | Voltage |
|------------|-----------|-------------|---------|
| VCC | Red | 5V rail | 5.0V |
| TRIG | Brown | STM32 PB1 | 3.3V (output) |
| ECHO | Cyan | STM32 PB7* | 5V (needs divider!) |
| GND | Black | Common GND | 0V |

**\*CRITICAL**: ECHO pins output 5V but STM32 GPIO is 3.3V tolerant!

---

### Voltage Divider for ECHO Pins

**Problem**: HC-SR04 ECHO outputs 5V, but STM32 GPIO max is 3.6V

**Solution**: Add voltage divider to each ECHO pin:

```
HC-SR04 ECHO → [1kΩ resistor] → STM32 GPIO (PB6 or PB7)
                            ↓
                      [2kΩ resistor]
                            ↓
                          GND
```

**Calculation**:
- Vout = Vin × (R2 / (R1 + R2))
- Vout = 5V × (2kΩ / (1kΩ + 2kΩ)) = 3.33V ✅

**Alternative**: Use 3.3V logic-compatible HC-SR04 modules (no divider needed)

---

### Ground Loop Issues

**Symptom**: Erratic readings, occasional 400cm spikes

**Causes**:
- Separate ground planes for STM32 and motors
- Long ground wires creating voltage drop

**Solutions**:
1. ✅ **Star ground topology**: All grounds meet at one point
2. ✅ **Twisted pair wiring**: Twist VCC/GND wires together
3. ✅ **Separate motor power**: Use optoisolator or separate power supply

**How to test**:
```bash
# Measure voltage between STM32 GND and sensor GND
# Should be < 10mV
```

---

### Signal Integrity Problems

**Symptom**: Sensors work when stationary, fail when motors running

**Causes**:
- EMI from motor brushes
- PWM noise on power rails
- Crosstalk between ECHO/TRIG lines

**Solutions**:
1. ✅ **Keep wires separated**: >2cm distance between motor and sensor wires
2. ✅ **Add ferrite beads**: On motor power lines
3. ✅ **Shielded cable**: For sensor ECHO/TRIG signals
4. ✅ **RC filter**: 100Ω + 100nF on each ECHO pin

---

## Software Troubleshooting

### Verify Firmware Configuration

Check pin configuration in `Core/Src/main.c`:

```c
// GPIO Configuration for Ultrasonic Sensors
GPIO_InitTypeDef GPIO_InitStruct = {0};

// Trigger pins (outputs)
GPIO_InitStruct.Pin = US_TRIG_A_PIN | US_TRIG_B_PIN;  // PB0, PB1
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

// Echo pins (inputs)
GPIO_InitStruct.Pin = US_ECHO_A_PIN | US_ECHO_B_PIN;  // PB6, PB7
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // Pull down
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

### Check DWT Cycle Counter

HC-SR04 timing requires microsecond precision:

```c
// In ultrasonic.c, verify DWT is initialized:
void Ultrasonic_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Test timing (should see ~29μs for 1cm)
uint32_t distance = Ultrasonic_MeasureA();
if (distance > 0 && distance < 400) {
    UART_SendString("Sensor A working!\r\n");
}
```

### Python Script Debug Output

The updated `rpi_motor_controller.py` now shows UART responses:

```python
# Start script and watch for debug output
python3 rpi_motor_controller.py

# Expected output every 500ms:
#   📩 STM32: US A=25cm B=30cm
#   📩 STM32: Wall L=25cm R=30cm

# If you see this instead:
#   ⚠️  WARNING: Both ultrasonic sensors reading 0cm (not connected?)
# → Hardware issue! Check wiring and power supply.
```

---

## Testing Procedure

### 1. Basic Hardware Test (No Code)

**Test Power Supply**:
```bash
# Measure sensor VCC with multimeter
Expected: 4.75V - 5.25V
If low: Check power supply capacity
```

**Test TRIG Signal**:
```bash
# Use logic analyzer or oscilloscope on PB0/PB1
Expected: 10μs HIGH pulse when 'U' command sent
If flat: STM32 not toggling pin (firmware issue)
```

**Test ECHO Response**:
```bash
# Use oscilloscope on ECHO pin (before voltage divider!)
Expected: HIGH pulse when object in range (58μs per cm)
If flat: Sensor not responding (power/wiring issue)
```

### 2. Firmware Debug Test

Enable verbose debug output:

```c
// In Core/Inc/main.h
#define ULTRASONIC_DEBUG 1

// Rebuild and flash
// UART output should show:
// "US A=25cm B=30cm" every 500ms
```

### 3. Python Integration Test

```python
# In rpi_motor_controller.py
python3 rpi_motor_controller.py

# Choose mode 1 (Interactive)
# Press 'U' to ping sensors
# Expected output:
#   📡 Ultrasonic ping sent
#   📩 STM32: US A=25.0cm B=30.0cm
#      Left: 25.0cm, Right: 30.0cm

# Press 'I' to see sensor status
# Expected:
#   Ultrasonic Left: 25.0cm
#   Ultrasonic Right: 30.0cm
#   Sensor Data Age: 0.5s
```

### 4. Continuous Monitoring Test

```python
# Sensors should update automatically every 500ms
# Watch for warnings:
#   ⚠️  WARNING: Left ultrasonic sensor reading 0cm
# → Check wiring to that specific sensor
```

---

## Common Error Messages

### "Both ultrasonic sensors reading 0cm (not connected?)"

**Meaning**: ECHO pins never go HIGH

**Causes**:
1. No 5V power to sensors
2. TRIG pins not connected/floating
3. ECHO pins not connected
4. Sensor facing metal surface too close (<2cm)

**Solution**: Check wiring with multimeter continuity test

---

### "WARNING: Left/Right ultrasonic sensor reading 0cm"

**Meaning**: One sensor failing, other working

**Causes**:
1. Loose connection on that sensor
2. Defective sensor module
3. ECHO voltage divider incorrect for that pin

**Solution**: Swap sensors to isolate hardware vs wiring issue

---

### "Ultrasonic data stale (>2s old)"

**Meaning**: No updates from STM32 for >2 seconds

**Causes**:
1. UART connection lost
2. STM32 stopped responding (crashed)
3. ultrasonicTask not running

**Solution**: 
1. Check UART connection (TX/RX swap?)
2. Reset STM32 (power cycle)
3. Check FreeRTOS task creation

---

### "Sensors read max distance (400cm)"

**Meaning**: ECHO timeout (no object detected)

**Causes**:
1. No obstacle within 4m range
2. Sensor facing wrong direction
3. Sound-absorbing material (fabric, foam)

**Solution**: Test with hard, flat surface at 10-50cm

---

## Physical Setup Guidelines

### Sensor Placement

```
Robot Top View:
┌─────────────────┐
│    HC-SR04 A    │  Left sensor (PB0/PB6)
│        🔊       │
│                 │
│   [STM32F401]   │
│                 │
│        🔊       │
│    HC-SR04 B    │  Right sensor (PB1/PB7)
└─────────────────┘
```

**Mounting Tips**:
- Point sensors outward at 45° angle
- Height: 10-15cm above ground
- Distance from edge: 2-3cm
- Avoid metal mounting brackets (reflections)

### Minimum Object Distance

HC-SR04 has blind spot at <2cm:
- ✅ Mount sensors high enough for ground clearance
- ✅ Angle sensors slightly downward (5-10°)
- ✅ Test at 5cm, 10cm, 20cm, 50cm distances

### Maximum Range

Effective range: 2cm - 400cm (theoretical)
- Reliable range: 5cm - 200cm
- Best accuracy: 10cm - 100cm

---

## Reference Values

### Expected Timing

| Measurement | Time | Notes |
|------------|------|-------|
| TRIG pulse | 10μs | From STM32 |
| ECHO min | 116μs | 2cm distance |
| ECHO max | 23.2ms | 400cm distance |
| Measurement interval | 50ms | Per sensor |
| Debug output | 500ms | UART print rate |

### Expected Distances

| Object | Distance | ECHO Time |
|--------|----------|-----------|
| Wall | 5-100cm | 290μs - 5.8ms |
| Hand | 3-30cm | 174μs - 1.74ms |
| Open space | >400cm | Timeout |

---

## Advanced Diagnostics

### Logic Analyzer Capture

Connect logic analyzer to:
1. PB0 (TRIG A)
2. PB6 (ECHO A before divider)
3. PA9 (UART TX)

Expected sequence:
```
PB0:  __|‾‾|_______________  (10μs HIGH)
         ↓
PB6:  _____|‾‾‾‾‾|________  (58μs per cm)
                  ↓
PA9:  "US A=XXcm\r\n"       (UART output)
```

### Oscilloscope Settings

- **Timebase**: 100μs/div
- **CH1 (TRIG)**: 5V/div, DC coupling
- **CH2 (ECHO)**: 5V/div, DC coupling
- **Trigger**: Rising edge on CH1 (TRIG)

### Multimeter Tests

```bash
# 1. Power supply voltage
VCC to GND: 5.0V ± 0.25V

# 2. TRIG pin idle state
PB0/PB1 to GND: 0V

# 3. ECHO pin with voltage divider
PB6/PB7 to GND: 0V (idle), 3.3V (active)

# 4. Common ground continuity
STM32 GND to Sensor GND: 0Ω (< 1Ω)
```

---

## Still Not Working?

### Replace Sensor Modules

HC-SR04 variants differ in quality:
- ✅ **Recommended**: HC-SR04+ (3.3V logic compatible)
- ✅ **Alternative**: US-100 (UART or GPIO mode)
- ❌ **Avoid**: No-name clones (inconsistent timing)

### Simplify Setup

Test with minimal configuration:
1. Remove motors (power only STM32 + sensors)
2. One sensor at a time
3. Short wires (<10cm)
4. Breadboard with solid connections

### Contact Support

If all else fails:
1. Post oscilloscope captures to forum
2. Include UART debug output
3. Photo of physical wiring
4. STM32 firmware version

---

## Quick Reference Card

```
PROBLEM               → CHECK THIS FIRST
════════════════════════════════════════════════════════
Both sensors = 0cm    → 5V power supply
One sensor = 0cm      → That sensor's ECHO wiring
Erratic readings      → Common ground, voltage divider
Works then fails      → Motor EMI, power supply noise
Always 400cm          → Sensor facing wrong way
No UART output        → ULTRASONIC_DEBUG = 1
Python shows no data  → UART wiring (TX↔RX swap?)
```

---

**Last Updated**: October 31, 2025  
**Firmware Version**: STM32F401 + FreeRTOS + HAL  
**Python Version**: 3.9+ (asyncio)
