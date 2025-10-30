# MX1508 Motor Controller with UART - STM32F401RC

## 📋 Project Overview

This project implements a UART-controlled motor driver system using the STM32F401RC microcontroller and MX1508 dual H-bridge motor driver. The robot can be controlled remotely via UART commands for forward, reverse, left, right, and stop operations.

---

## 🔌 Hardware Connections

### Motor Driver Connections (MX1508)
| STM32 Pin | MX1508 Pin | Function | Description |
|-----------|------------|----------|-------------|
| PA0 | IN1 | Motor 1 Forward | Left Motor Forward |
| PA1 | IN2 | Motor 1 Reverse | Left Motor Reverse |
| PA2 | IN3 | Motor 2 Forward | Right Motor Forward |
| PA3 | IN4 | Motor 2 Reverse | Right Motor Reverse |

### UART Connections (USART1)
| STM32 Pin | Function | Connection |
|-----------|----------|------------|
| PA9 | USART1_TX | Connect to RX of USB-Serial adapter |
| PA10 | USART1_RX | Connect to TX of USB-Serial adapter |
| GND | Ground | Common ground with serial adapter |

### Power Connections
- **STM32**: 3.3V/5V power supply
- **MX1508**: 2V-10V motor power supply (separate from logic)
- **Common Ground**: Connect all grounds together

---

## ⚙️ UART Configuration

### Baud Rate Calculation
```
System Clock (HSI): 16 MHz
APB2 Clock: 16 MHz (USART1 is on APB2)
Desired Baud Rate: 9600 bps

Formula: BRR = fCK / Baud
BRR = 16,000,000 / 9600 = 1666.67
BRR (hex) = 0x0682

Actual Baud Rate = 16,000,000 / 1666 = 9603.84 bps
Error = 0.04% (Acceptable - should be < 2%)
```

### UART Settings
- **Baud Rate**: 9600 bps
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None
- **Mode**: Asynchronous

### Alternate Function Configuration
- **PA9**: AF7 (USART1_TX)
- **PA10**: AF7 (USART1_RX)

---

## 🎮 Command Protocol

### Command Characters
| Command | Character | ASCII | Hex | Action |
|---------|-----------|-------|-----|--------|
| Forward | `F` | 70 | 0x46 | Both motors forward |
| Reverse | `R` | 82 | 0x52 | Both motors backward |
| Left | `L` | 76 | 0x4C | Left motor reverse, right forward |
| Right | `T` | 84 | 0x54 | Left motor forward, right reverse |
| Stop | `S` | 83 | 0x53 | Both motors stop |

### Motor Control Logic

#### Forward Movement
```
Motor 1 (Left):  IN1=HIGH, IN2=LOW
Motor 2 (Right): IN3=HIGH, IN4=LOW
```

#### Reverse Movement
```
Motor 1 (Left):  IN1=LOW, IN2=HIGH
Motor 2 (Right): IN3=LOW, IN4=HIGH
```

#### Left Turn (Spot Turn)
```
Motor 1 (Left):  IN1=LOW, IN2=HIGH (Reverse)
Motor 2 (Right): IN3=HIGH, IN4=LOW (Forward)
```

#### Right Turn (Spot Turn)
```
Motor 1 (Left):  IN1=HIGH, IN2=LOW (Forward)
Motor 2 (Right): IN3=LOW, IN4=HIGH (Reverse)
```

#### Stop
```
Motor 1 (Left):  IN1=LOW, IN2=LOW
Motor 2 (Right): IN3=LOW, IN4=LOW
```

---

## 🛡️ Safety Features

### 1. **Emergency Stop Timeout**
- If no command received for **2000 ms** (2 seconds)
- Automatically stops all motors
- Prevents runaway robot

### 2. **UART Error Handling**
- Overrun Error (ORE) detection
- Framing Error (FE) detection
- Noise Error (NE) detection
- Parity Error (PE) detection
- Automatic error flag clearing
- Optional motor stop on error

### 3. **Initialization Safety**
- All motor pins initialized to LOW (stopped)
- Motors stopped at startup

### 4. **Invalid Command Handling**
- Unknown commands trigger motor stop
- Prevents undefined behavior

---

## 🧪 Testing Procedures

### Test 1: GPIO Configuration Verification
**Objective**: Verify motor control pins are configured correctly

**Steps**:
1. Power up the STM32
2. Check PA0-PA3 with multimeter/oscilloscope
3. All pins should be LOW (0V) at startup
4. Expected: All motor pins in stopped state

**Pass Criteria**: All pins read 0V initially

---

### Test 2: UART Loopback Test
**Objective**: Verify UART transmission and reception

**Setup**:
1. Connect PA9 (TX) to PA10 (RX) with a jumper wire
2. Add debug code to transmit received characters

**Test Code** (Add to main loop for testing):
```c
// Loopback test - echo received character
if (new_command) {
    // Transmit back the received character
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = rx_data;
    new_command = false;
}
```

**Steps**:
1. Send character 'A' via serial terminal
2. Should receive 'A' back
3. Repeat with other characters

**Pass Criteria**: All sent characters are echoed back

---

### Test 3: Individual Motor Control
**Objective**: Test each motor direction independently

**Test Sequence**:
1. **Stop Command** - Send `S`
   - Expected: All pins LOW
   
2. **Forward Command** - Send `F`
   - Expected: PA0=HIGH, PA1=LOW, PA2=HIGH, PA3=LOW
   - Motors should run forward
   
3. **Stop Command** - Send `S`
   - Expected: All pins LOW
   
4. **Reverse Command** - Send `R`
   - Expected: PA0=LOW, PA1=HIGH, PA2=LOW, PA3=HIGH
   - Motors should run backward
   
5. **Left Turn** - Send `L`
   - Expected: PA0=LOW, PA1=HIGH, PA2=HIGH, PA3=LOW
   
6. **Right Turn** - Send `T`
   - Expected: PA0=HIGH, PA1=LOW, PA2=LOW, PA3=HIGH

**Pass Criteria**: All motor combinations work correctly

---

### Test 4: UART Communication Range
**Objective**: Test UART at specified baud rate

**Equipment**: 
- USB-to-Serial adapter (FTDI, CP2102, CH340, etc.)
- Serial terminal (PuTTY, Tera Term, Arduino Serial Monitor)

**Settings**:
- Baud: 9600
- Data: 8 bits
- Stop: 1 bit
- Parity: None

**Steps**:
1. Connect USB-Serial adapter to PA9/PA10
2. Open serial terminal with correct settings
3. Send commands: F, R, L, T, S
4. Observe motor response

**Pass Criteria**: All commands execute correctly

---

### Test 5: Error Handling
**Objective**: Verify UART error detection

**Test Cases**:

**A. Overrun Error**
- Send rapid bursts of characters
- System should handle without crash
- Motor should respond or stop safely

**B. Framing Error**
- Send data at wrong baud rate (e.g., 4800)
- System should detect framing error
- Motors should stop or maintain safe state

**C. Invalid Commands**
- Send invalid characters (e.g., '1', '2', 'X', 'Y')
- Expected: Motors stop on invalid command

**Pass Criteria**: System handles all errors gracefully

---

### Test 6: Safety Timeout
**Objective**: Verify emergency stop timeout

**Steps**:
1. Send `F` command (motors start forward)
2. Wait for 2 seconds without sending any command
3. Motors should automatically stop
4. LED indicator (if added) should show timeout

**Pass Criteria**: Motors stop after 2 seconds of inactivity

---

### Test 7: Continuous Operation
**Objective**: Test system stability

**Steps**:
1. Send alternating commands for 5 minutes:
   - F, S, R, S, L, S, T, S (repeat)
2. Send command every 500ms
3. Monitor for crashes or unexpected behavior

**Pass Criteria**: System runs continuously without errors

---

## 🔧 Debugging Tools

### 1. **LED Indicators** (Optional Enhancement)
Add LEDs to visualize system state:

```c
// Add to GPIO_Init()
#define LED_RX_PIN    GPIO_PIN_13  // LED for RX activity
#define LED_ERROR_PIN GPIO_PIN_14  // LED for errors
#define LED_TIMEOUT_PIN GPIO_PIN_15  // LED for timeout

// Toggle in ISR and main loop
HAL_GPIO_TogglePin(GPIOC, LED_RX_PIN);  // RX activity
```

### 2. **Serial Debug Output**
Add debug transmission:

```c
void UART_SendString(char* str) {
    while (*str) {
        while (!(USART1->SR & USART_SR_TXE));
        USART1->DR = *str++;
    }
}

// Usage
UART_SendString("Motor Forward\r\n");
```

### 3. **Register Inspection**
Check USART1 registers in debugger:
- **USART1->SR**: Status register
- **USART1->DR**: Data register
- **USART1->BRR**: Baud rate register (should be 0x0682)
- **USART1->CR1**: Control register 1 (should be 0x202C)

---

## 📊 Register Configuration Summary

### USART1 Configuration Registers

| Register | Value | Description |
|----------|-------|-------------|
| **BRR** | 0x0682 | Baud rate = 9600 @ 16MHz |
| **CR1** | 0x202C | UE=1, RE=1, TE=1, RXNEIE=1 |
| **CR2** | 0x0000 | 1 stop bit |
| **CR3** | 0x0001 | EIE=1 (Error interrupt) |

### GPIO Configuration

| Pin | Mode | AF | Pull | Speed |
|-----|------|----|----|-------|
| PA0-PA3 | Output PP | - | None | Low |
| PA9 | AF PP | AF7 | Pull-up | High |
| PA10 | AF PP | AF7 | Pull-up | High |

---

## 🚨 Troubleshooting

### Problem: No UART reception
**Check**:
1. PA9/PA10 properly connected (TX↔RX swapped)
2. Common ground connected
3. Baud rate matches (9600)
4. USART1 clock enabled
5. NVIC interrupt enabled

### Problem: Motors not responding
**Check**:
1. Motor power supply connected
2. MX1508 powered correctly
3. PA0-PA3 connections correct
4. Motor driver not damaged
5. Commands being received (check with loopback)

### Problem: Erratic behavior
**Check**:
1. UART errors occurring (check uart_error flag)
2. Noise on communication lines
3. Power supply stable
4. Ground connections solid

### Problem: Motors don't stop
**Check**:
1. Safety timeout working (2 second test)
2. Stop command received correctly
3. GPIO pins actually going LOW

---

## 🔬 Advanced Testing

### Oscilloscope Verification
1. **UART Signals**: Should see clean digital signals at 9600 baud
2. **Motor PWM**: Can be added for speed control later
3. **Timing**: Verify safety timeout is exactly 2 seconds

### Logic Analyzer
1. Capture UART frame structure
2. Verify start bit, data bits, stop bit
3. Check for glitches or timing issues

---

## 📈 Future Enhancements

1. **PWM Speed Control**: Use TIM2/TIM3 for variable speed
2. **Acceleration/Deceleration**: Smooth motor startup/stop
3. **Multiple Command Format**: Support multi-byte commands
4. **Status Feedback**: Send acknowledgments back to controller
5. **Battery Monitoring**: ADC to monitor battery voltage
6. **Obstacle Detection**: Add sensors for autonomous operation

---

## 📝 Code Quality Checklist

- [✓] USART peripheral chosen correctly (USART1)
- [✓] Pins configured with correct AF (AF7)
- [✓] BRR calculated correctly for 9600 baud @ 16MHz
- [✓] Clock enabled (RCC APB2 for USART1)
- [✓] CR1, CR2, CR3 configured properly
- [✓] NVIC interrupt enabled
- [✓] ISR handler implemented with error handling
- [✓] Communication protocol defined and documented
- [✓] Test plan prepared
- [✓] Safety features implemented (timeout, emergency stop)

---

## 📚 Reference Documents

- **STM32F401xB/C Datasheet**: Pin configurations and electrical specs
- **STM32F401 Reference Manual**: USART register details (RM0368)
- **MX1508 Datasheet**: Motor driver specifications
- **HAL Library Documentation**: STM32F4 HAL functions

---

## 🎯 Quick Start Guide

1. **Build Project**: Compile in STM32CubeIDE or your preferred IDE
2. **Flash**: Program STM32F401RC with generated .hex/.bin file
3. **Connect Hardware**: Wire motors and UART adapter as per diagram
4. **Open Terminal**: Use serial terminal at 9600 baud
5. **Test Commands**: Send F, R, L, T, S to control robot
6. **Safety Check**: Verify auto-stop after 2 seconds

---

## ⚠️ Important Notes

1. **Separate Power**: Use separate power for motors and logic if motors draw > 500mA
2. **Ground Connection**: Always connect common ground between STM32 and motor driver
3. **Protection Diodes**: MX1508 has built-in protection, but external diodes recommended for large motors
4. **Current Limit**: MX1508 max 1.5A per channel - don't exceed
5. **Voltage Range**: Keep motor voltage within 2V-10V range

---

## 📞 Support

For issues or questions:
- Review this documentation thoroughly
- Check connections with multimeter
- Use loopback test to isolate UART issues
- Verify clock configuration matches documentation

---

**Version**: 1.0  
**Date**: October 30, 2025  
**MCU**: STM32F401RCT6  
**Motor Driver**: MX1508  
**IDE**: STM32CubeIDE
