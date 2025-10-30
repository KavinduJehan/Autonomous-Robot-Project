# Hardware Wiring Diagram - MX1508 Motor Controller

## 📌 Complete Pin Mapping

### STM32F401RC Pinout

```
                    STM32F401RCT6 (LQFP64)
                    
                         ┌─────────┐
                    VBAT │1     64 │ VDDA
                    PC13 │2     63 │ PC4
                    PC14 │3     62 │ PC5
                    PC15 │4     61 │ PB0
                     PH0 │5     60 │ PB1
                     PH1 │6     59 │ PB2
                    NRST │7     58 │ PB10
                    PC0  │8     57 │ VCAP1
                    PC1  │9     56 │ VSS
                    PC2  │10    55 │ VDD
                    PC3  │11    54 │ PB12
                    VSSA │12    53 │ PB13
                    VDDA │13    52 │ PB14
              [TX]  PA0  │14    51 │ PB15 [IN4]
              [RX]  PA1  │15    50 │ PC6
              [RX]  PA2  │16    49 │ PC7
              [RX]  PA3  │17    48 │ PC8
                    VSS  │18    47 │ PC9
                    VDD  │19    46 │ PA8
                    PA4  │20    45 │ PA9  [USART1_TX] ←
                    PA5  │21    44 │ PA10 [USART1_RX] ←
                    PA6  │22    43 │ PA11
                    PA7  │23    42 │ PA12
                    PC4  │24    41 │ PA13 [SWDIO]
                    PC5  │25    40 │ VSS
                    PB0  │26    39 │ VDD
                    PB1  │27    38 │ PA14 [SWCLK]
                    PB2  │28    37 │ PA15
                   PB10  │29    36 │ PC10
                   VCAP1 │30    35 │ PC11
                    VSS  │31    34 │ PC12
                    VDD  │32    33 │ PD2
                         └─────────┘

KEY PINS:
  PA0  - Motor 1 IN1 (Forward)
  PA1  - Motor 1 IN2 (Reverse)
  PA2  - Motor 2 IN3 (Forward)
  PA3  - Motor 2 IN4 (Reverse)
  PA9  - USART1 TX
  PA10 - USART1 RX
```

---

## 🔌 MX1508 Motor Driver Pinout

```
        MX1508 Dual H-Bridge Motor Driver
        
        ┌─────────────────────────┐
        │                         │
    IN1 │●  ┌───┐                │ MOTOR A+
    IN2 │●  │   │                │ MOTOR A-
        │   │ M │                │
    VCC │●  │ X │                │ MOTOR B+
    GND │●  │ 1 │                │ MOTOR B-
        │   │ 5 │                │
    IN3 │●  │ 0 │                │
    IN4 │●  │ 8 │                │
        │   └───┘                │
        └─────────────────────────┘

Pin Functions:
  IN1  - Motor A forward control
  IN2  - Motor A reverse control
  IN3  - Motor B forward control
  IN4  - Motor B reverse control
  VCC  - Motor power supply (2V-10V, max 1.5A per channel)
  GND  - Ground
```

---

## 🔗 Connection Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                    COMPLETE WIRING DIAGRAM                       │
└──────────────────────────────────────────────────────────────────┘


    USB-TTL Adapter              STM32F401RC              MX1508
    ┌──────────┐               ┌────────────┐          ┌─────────┐
    │          │               │            │          │         │
    │   VCC ●──┼──X            │            │   PA0 ●──┼──● IN1 │
    │          │  (Optional)   │            │          │         │
    │          │               │            │   PA1 ●──┼──● IN2 │
    │   TXD ●──┼───────────────┼──● PA10    │          │         │
    │          │               │   (RX)     │   PA2 ●──┼──● IN3 │
    │          │               │            │          │         │
    │   RXD ●──┼───────────────┼──● PA9     │   PA3 ●──┼──● IN4 │
    │          │               │   (TX)     │          │         │
    │          │               │            │          │   VCC ●─┼──┐
    │   GND ●──┼───────┬───────┼──● GND     │          │         │  │
    │          │       │       │            │          │   GND ●─┼──┤
    └──────────┘       │       └────────────┘          └─────────┘  │
                       │                                             │
                       │                                             │
                       │       ┌─────────────────┐                  │
                       │       │  Motor Power    │                  │
                       │       │  Supply         │                  │
                       │       │  (6-9V)         │                  │
                       │       │                 │                  │
                       └───────┼──● GND (-)      │                  │
                               │                 │                  │
                               │         VCC (+)●┼──────────────────┘
                               └─────────────────┘


    Motor A (Left)              Motor B (Right)
    ┌──────────┐               ┌──────────┐
    │    DC    │               │    DC    │
    │   Motor  │               │   Motor  │
    │          │               │          │
    │    +  ●──┼───────────────┼──● A+    │
    │       │  │               │          │
    │    -  ●──┼───────────────┼──● A-    │
    │          │               │          │
    └──────────┘               └──────────┘
                                     │
                                     │
                               ┌─────┴─────┐
                               │     B+  ●──┼────●  +
                               │            │
                               │     B-  ●──┼────●  -
                               └───────────┘
```

---

## 📐 Breadboard Layout

```
                 Power Rails
                 +    -
                 │    │
    ┌────────────┼────┼─────────────────────────────┐
    │ USB-TTL    │    │                             │
    │  Adapter   │    │         STM32F401RC         │
    │ ┌────┐     │    │        ┌──────────┐         │
    │ │USB │     │    └────────┤ GND      │         │
    │ └─┬──┘     │             │          │         │
    │   │ TXD────┼─────────────┤ PA10(RX) │         │
    │   │        │             │          │         │
    │   │ RXD────┼─────────────┤ PA9(TX)  │         │
    │   │        │             │          │         │
    │   │ GND────┼─────┬───────┤ GND      │         │
    │   │        │     │       │          │         │
    │            │     │       │ PA0      ├─────────┼─→ IN1
    │            │     │       │ PA1      ├─────────┼─→ IN2
    │            │     │       │ PA2      ├─────────┼─→ IN3
    │            │     │       │ PA3      ├─────────┼─→ IN4
    │            │     │       │          │         │
    │            │     │       │ 3.3V/5V  ├─────────┼─→ +3.3V Rail
    │            │     │       └──────────┘         │
    │            │     │                            │
    │            │     │       MX1508               │
    │            │     │      ┌────────┐            │
    │            │     │  IN1 │        │ A+         │
    │            │     │  IN2 │        │ A-         │
    │            │     │  VCC │        │            │
    │            │     └──GND─┤        │ B+         │
    │            │            │        │ B-         │
    │            │       IN3 │        │            │
    │            │       IN4 │        │            │
    │            │            └────────┘            │
    │            │                │                 │
    │            │                │                 │
    │  Battery   │     (6-9V)     │                 │
    │  ┌───┐     │                │                 │
    │  │ + ├─────┼────────────────┘                 │
    │  │   │     │                                  │
    │  │ - ├─────┤                                  │
    │  └───┘     │                                  │
    └────────────┴──────────────────────────────────┘
```

---

## 📋 Connection Table

| Function | STM32 Pin | MX1508 Pin | Wire Color (Suggested) |
|----------|-----------|------------|------------------------|
| Motor 1 Forward | PA0 | IN1 | Yellow |
| Motor 1 Reverse | PA1 | IN2 | Orange |
| Motor 2 Forward | PA2 | IN3 | Green |
| Motor 2 Reverse | PA3 | IN4 | Blue |
| UART TX | PA9 | - | White |
| UART RX | PA10 | - | Gray |
| STM32 Ground | GND | GND | Black |
| Motor Power | - | VCC | Red |
| Motor Ground | - | GND | Black |

---

## 🔋 Power Supply Guidelines

### Option 1: Separate Supplies (Recommended)
```
USB Power (5V)
    └──→ STM32F401RC (Logic)

Battery Pack (6-9V)
    └──→ MX1508 VCC (Motors)

Common Ground: All grounds connected together
```

### Option 2: Single Supply with Regulator
```
Battery Pack (7.4V-9V)
    ├──→ MX1508 VCC (Motors)
    └──→ 5V Regulator → STM32F401RC

Still need common ground!
```

### Current Requirements
- **STM32F401RC**: ~50mA (logic)
- **MX1508**: Up to 1.5A per channel (motor dependent)
- **Total**: Size power supply for motor current + 50mA margin

---

## ⚡ Safety Connections

### 1. Decoupling Capacitors
```
Add near power pins:
- 100nF ceramic capacitor across VDD-VSS (STM32)
- 10µF electrolytic capacitor across motor power supply
- 100µF electrolytic capacitor across MX1508 VCC-GND
```

### 2. Flyback Diodes
```
MX1508 has built-in protection, but for extra safety:
- 1N4007 diodes across motor terminals (cathode to +)
```

### 3. Pull-up Resistors
```
Optional for UART lines if needed:
- 10kΩ resistor from PA9 to 3.3V
- 10kΩ resistor from PA10 to 3.3V
```

---

## 🔍 Testing Points

### Voltage Check Points
1. **STM32 VDD**: Should read 3.3V or 5V
2. **MX1508 VCC**: Should read motor supply voltage (6-9V)
3. **PA0-PA3**: Should read 0V (stopped) or 3.3V (active)
4. **PA9, PA10**: Should read 3.3V idle, toggling during transmission

### Continuity Checks (Power OFF)
1. Check all ground connections are common
2. Verify no shorts between VCC and GND
3. Confirm motor connections are not shorted

---

## 🛠️ Assembly Instructions

### Step 1: Prepare Components
- STM32F401RC development board
- MX1508 motor driver module
- USB-TTL serial adapter
- 2× DC motors (6V-9V rating)
- Breadboard and jumper wires
- 6V-9V battery pack

### Step 2: Mount STM32
1. Insert STM32 board into breadboard
2. Ensure pins are not bent
3. Connect power rails

### Step 3: Connect Motor Driver
1. Place MX1508 on breadboard
2. Connect GND to common ground rail
3. Connect VCC to motor power supply
4. Connect control pins (IN1-IN4) to PA0-PA3

### Step 4: Connect UART
1. Connect USB-TTL adapter TX to PA10
2. Connect USB-TTL adapter RX to PA9
3. Connect USB-TTL adapter GND to common ground

### Step 5: Connect Motors
1. Connect Motor A (left) to A+/A- terminals
2. Connect Motor B (right) to B+/B- terminals
3. Verify motor polarity (both should spin same direction on forward)

### Step 6: Power Up Sequence
1. Connect STM32 power first (USB or regulated supply)
2. Verify 3.3V on VDD pin
3. Connect motor power supply
4. Verify no smoke or excessive heat
5. Send 'S' command to verify communication

---

## 📸 Photo Reference Points

When documenting your build, photograph:
1. Overall breadboard layout
2. Close-up of STM32 connections
3. Close-up of MX1508 connections
4. Motor connections
5. UART adapter connections
6. Power supply connections

---

## 🚨 Troubleshooting Wiring

### No Communication
- **Check**: TX ↔ RX swap (STM32 TX goes to adapter RX)
- **Check**: Common ground connection
- **Check**: 3.3V/5V compatibility of USB-TTL adapter

### Motors Don't Move
- **Check**: Motor power supply voltage (6-9V)
- **Check**: MX1508 VCC connection
- **Check**: Ground connection between STM32 and MX1508
- **Check**: Control signal connections (PA0-PA3 to IN1-IN4)

### Motors Run Continuously
- **Check**: Control pins not floating (should be driven by STM32)
- **Check**: Code running correctly
- **Check**: Send 'S' command

### One Motor Works, Other Doesn't
- **Check**: All four control pins connected
- **Check**: Motor B connections
- **Check**: MX1508 not damaged

---

## ✅ Final Verification Checklist

Before powering on:
- [ ] All ground connections verified with multimeter (continuity)
- [ ] No shorts between VCC and GND on any module
- [ ] Motor power supply polarity correct
- [ ] UART TX/RX properly crossed
- [ ] STM32 power supply correct voltage (3.3V or 5V)
- [ ] Motor driver power supply in range (2-10V)
- [ ] All control pins connected (IN1-IN4)
- [ ] Decoupling capacitors in place
- [ ] Motors free to spin (not mechanically jammed)

---

## 📚 Additional Resources

- **STM32F401RC Pinout**: [STMicroelectronics Website]
- **MX1508 Datasheet**: Search "MX1508 datasheet PDF"
- **USB-TTL Adapter Drivers**: Check manufacturer website (FTDI, CP2102, CH340)

---

**Document Version**: 1.0  
**Last Updated**: October 30, 2025
