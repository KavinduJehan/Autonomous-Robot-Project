# Quick Reference Card - Motor Controller

## 🎯 Quick Start

### 1. Upload Code
```bash
# Build and flash in STM32CubeIDE or:
arm-none-eabi-gcc ... (or use your build system)
```

### 2. Connect Hardware
```
STM32    →  MX1508
PA0      →  IN1
PA1      →  IN2
PA2      →  IN3
PA3      →  IN4
GND      →  GND

STM32    →  USB-TTL
PA9(TX)  →  RX
PA10(RX) →  TX
GND      →  GND
```

### 3. Test
```bash
# Install Python package
pip install pyserial

# Run test script
python uart_test.py COM3  # Windows
python uart_test.py /dev/ttyUSB0  # Linux
```

---

## 📡 UART Settings

| Parameter | Value |
|-----------|-------|
| Baud Rate | 9600 |
| Data Bits | 8 |
| Stop Bits | 1 |
| Parity | None |
| Flow Control | None |

---

## 🎮 Commands

### Movement Commands
| Key | Hex | Action | Behavior |
|-----|-----|--------|----------|
| `F` | 0x46 | Forward | Both motors forward at current speed |
| `R` | 0x52 | Reverse | Both motors backward at current speed |
| `L` | 0x4C | Left Turn | Left reverse, right forward (spot turn) |
| `T` | 0x54 | Right Turn | Left forward, right reverse (spot turn) |
| `S` | 0x53 | Stop | All motors stop (PWM = 0%) |

### Speed Control Commands (PWM)
| Key | Hex | Speed Level | PWM Duty | Use Case |
|-----|-----|-------------|----------|----------|
| `1` | 0x31 | SLOW | 40% | Precise movements, wall following |
| `2` | 0x32 | MEDIUM | 70% | Normal operation (default) |
| `3` | 0x33 | FAST | 100% | Maximum speed |

**Note**: Speed commands change the speed for subsequent movement commands.

---

## 🔧 Register Values

### TIM3 PWM Configuration
```c
Prescaler:  15        // 16MHz / (15+1) = 1MHz timer clock
Period:     999       // 1MHz / (999+1) = 1kHz PWM
Frequency:  1 kHz     // PWM frequency
Channels:   CH1-CH4   // PA0-PA3
Resolution: 0-100%    // Duty cycle control
```

### USART1 Registers
```c
BRR  = 0x0682  // Baud rate: 9600 @ 16MHz
CR1  = 0x202C  // UE=1, RE=1, TE=1, RXNEIE=1
CR2  = 0x0000  // 1 stop bit
CR3  = 0x0001  // Error interrupt enable
```

### Clock Configuration
```
HSI:        16 MHz (internal)
SYSCLK:     16 MHz
AHB:        16 MHz
APB1:       16 MHz
APB2:       16 MHz (USART1 clock)
```

---

## 🛡️ Safety Features

| Feature | Value | Description |
|---------|-------|-------------|
| Command Timeout | 2000 ms | Auto-stop if no command |
| Error Detection | All | ORE, FE, NE, PE |
| Invalid Command | Stop | Unknown cmd = motor stop |
| Init State | Stopped | All motors off at boot |

---

## 🧪 Quick Tests

### Test 1: Loopback
```
1. Connect PA9 to PA10 with wire
2. Send 'A' in terminal
3. Should receive 'A' back
```

### Test 2: Motor Test with Speed Control
```
Send: 2  → Set medium speed (70%)
Send: F  → Motors forward at 70%
Send: S  → Motors stop
Send: 1  → Set slow speed (40%)
Send: R  → Motors reverse at 40%
Send: S  → Motors stop
Send: 3  → Set fast speed (100%)
Send: L  → Left turn at 100%
Send: T  → Right turn at 100%
Send: S  → Final stop
```

### Test 3: Timeout
```
Send: F
Wait: 2+ seconds
Result: Motors auto-stop
```

---

## 💡 Terminal Commands

### PuTTY Setup
```
Connection Type: Serial
Speed: 9600
Data bits: 8
Stop bits: 1
Parity: None
Flow control: None
```

### Arduino Serial Monitor
```
Baud: 9600
Line ending: No line ending
```

### Python Script
```bash
# Windows
python uart_test.py COM3

# Linux
python uart_test.py /dev/ttyUSB0
```

---

## ⚠️ Common Issues

### Issue: No UART response
**Fix**: 
- Check TX↔RX crossed
- Verify common ground
- Check baud rate = 9600

### Issue: Motors don't move
**Fix**:
- Check motor power supply (6-9V)
- Verify PA0-PA3 connections
- Send 'S' then 'F' command

### Issue: One motor works
**Fix**:
- Check all 4 wires (IN1-IN4)
- Verify both motors powered
- Test motor directly with battery

---

## 📊 Pin Summary

```
Motor Control PWM:  PA0 (TIM3_CH1), PA1 (TIM3_CH2), 
                    PA2 (TIM3_CH3), PA3 (TIM3_CH4)
UART:               PA9(TX), PA10(RX)
LED Indicators:     PC13 (RX), PC14 (TX), PB12 (Heartbeat)
Alternate Func:     AF2 (TIM3), AF7 (USART1)
Ground:             GND (common)
```

---

## 🔋 Power Requirements

| Component | Voltage | Current |
|-----------|---------|---------|
| STM32F401RC | 3.3-5V | ~50mA |
| MX1508 Logic | 3.3-5V | ~10mA |
| MX1508 Motors | 2-10V | 1.5A/channel |

**Note**: Use separate supply for motors if > 500mA

---

## 🎨 LED Indicators (Optional)

Add LEDs for visual feedback:
```c
PC13: RX Activity (toggle in ISR)
PC14: Error Status (on when error)
PC15: Timeout Status (on when timeout)
```

---

## 📝 Modifying Commands

To change commands, edit in `main.h`:
```c
// Movement Commands
#define CMD_FORWARD       'F'  // Change to your char
#define CMD_REVERSE       'R'
#define CMD_LEFT          'L'
#define CMD_RIGHT         'T'
#define CMD_STOP          'S'

// Speed Commands
#define CMD_SPEED_SLOW    '1'  // 40% PWM
#define CMD_SPEED_MEDIUM  '2'  // 70% PWM
#define CMD_SPEED_FAST    '3'  // 100% PWM

// Speed Levels (percentage)
#define SPEED_SLOW        40
#define SPEED_MEDIUM      70
#define SPEED_FAST        100
```

---

## 🔄 Changing Baud Rate

To change to 115200 baud:
```c
// In USART1_Init():
// BRR = 16000000 / 115200 = 138.89 ≈ 0x008B
USART1->BRR = 0x008B;  

// Update header:
#define UART_BAUDRATE     115200
```

---

## ⚙️ PWM Tuning

### Changing PWM Frequency
```c
// In main.h:
#define PWM_PRESCALER     15    // Change for different freq
#define PWM_PERIOD        999   

// Formula: PWM_freq = 16MHz / ((PSC+1) * (PERIOD+1))
// Example: 16MHz / (16 * 1000) = 1 kHz

// For 10 kHz PWM:
#define PWM_PRESCALER     15    // 16MHz / 16 = 1MHz
#define PWM_PERIOD        99    // 1MHz / 100 = 10kHz
```

### Custom Speed Levels
```c
// In main.h, adjust percentage values:
#define SPEED_SLOW        30    // Very slow (30%)
#define SPEED_MEDIUM      60    // Medium (60%)
#define SPEED_FAST        90    // Fast but not max (90%)

// Or add new levels:
#define SPEED_CRAWL       20    // Ultra slow
#define SPEED_TURBO       100   // Maximum
```

### Testing PWM Output
Use oscilloscope or multimeter:
- **1 kHz PWM at 50%**: Average voltage = 1.65V (3.3V × 0.5)
- **1 kHz PWM at 70%**: Average voltage = 2.31V (3.3V × 0.7)
- Connect probe to PA0-PA3 and GND

---

## 🎯 Troubleshooting Flowchart

```
No Response?
    ├─→ Check power (3.3V on STM32)
    ├─→ Check TX/RX swap
    ├─→ Check ground connection
    ├─→ Verify baud = 9600
    └─→ Test with loopback

Motors Don't Run?
    ├─→ Check motor power (6-9V)
    ├─→ Send 'S' then 'F'
    ├─→ Verify PA0-PA3 connections
    ├─→ Test with LED on PA0
    └─→ Check MX1508 not damaged

Erratic Behavior?
    ├─→ Check for UART errors
    ├─→ Verify clean power supply
    ├─→ Add decoupling capacitors
    └─→ Reduce wire length
```

---

## 📱 Mobile Control (Future)

Can add Bluetooth module:
```
HC-05/HC-06 → USART2 (PA2/PA3)
Configure same baud: 9600
Use same command protocol
```

---

## 🚀 Performance Notes

| Metric | Value |
|--------|-------|
| Command latency | ~1-2 ms |
| Main loop freq | ~100 Hz |
| UART interrupt priority | 5 |
| Max command rate | ~1000 Hz |

---

## 📖 File Structure

```
motor controller with uart/
├── Core/
│   ├── Inc/
│   │   └── main.h           # Pin definitions, prototypes
│   └── Src/
│       ├── main.c           # Main code, UART, motor control
│       └── stm32f4xx_it.c   # Interrupt handlers
├── Drivers/                 # HAL/CMSIS libraries
├── MOTOR_CONTROLLER_README.md  # Full documentation
├── WIRING_DIAGRAM.md           # Hardware connections
├── uart_test.py                # Python test script
└── QUICK_REFERENCE.md          # This file
```

---

## ✅ Pre-Flight Checklist

Before first power-on:
- [ ] Code compiled without errors
- [ ] Code flashed to STM32
- [ ] All connections verified
- [ ] Ground common across all modules
- [ ] Motor power supply correct voltage
- [ ] No shorts between power/ground
- [ ] USB-TTL adapter recognized by PC
- [ ] Motors can spin freely

---

## 🆘 Emergency Stop

### Hardware:
- Disconnect motor power supply

### Software:
1. Send 'S' command
2. Or wait 2 seconds (auto-stop)
3. Reset STM32 (NRST button)

---

## 📞 Support Resources

- **README**: Full documentation (`MOTOR_CONTROLLER_README.md`)
- **Wiring**: Connection diagrams (`WIRING_DIAGRAM.md`)
- **Test Script**: Python testing tool (`uart_test.py`)
- **Code**: Well-commented source (`main.c`, `main.h`)

---

**Keep This Card Handy While Testing!**

Print or keep open in second monitor for quick reference during development and testing.
