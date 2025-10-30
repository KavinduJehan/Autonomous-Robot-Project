# LED Indicators for UART Activity

## ⚠️ IMPORTANT: Why You Shouldn't Connect LEDs to UART Pins Directly

### The Problem with PA9/PA10

You noticed different voltage levels on PA9 and PA10 because:

1. **PA9 (USART TX)**: 
   - Alternate Function Output mode
   - Actively transmitting data (toggling HIGH/LOW)
   - **Average voltage appears lower** when constantly switching
   - Output driver optimized for UART, not LED driving

2. **PA10 (USART RX)**:
   - Alternate Function Input mode with **Pull-Up enabled**
   - Internal pull-up resistor (~40kΩ) keeps voltage HIGH
   - Not actively driving, just monitoring
   - **Appears to have higher/more stable voltage**

### Why This Is Dangerous

```
⚠️  CRITICAL ISSUES:

1. No Current Limiting
   - STM32 GPIO max: 25mA (absolute max)
   - Recommended max: 8-12mA
   - LED without resistor: 20-40mA
   → RISK: Pin damage, malfunction, reduced lifespan

2. UART Signal Integrity
   - LED load affects signal quality
   - Can cause communication errors
   - May prevent proper UART operation

3. Voltage Drop
   - LED forward voltage: ~2V
   - Remaining for logic: 1.3V
   - May not meet UART threshold (1.4V min)
```

---

## ✅ Proper Solution: Dedicated LED Indicators

### Updated Pin Assignment

I've modified the code to use **dedicated GPIO pins** for LED indicators:

```
Original (WRONG):
PA9  (UART TX) → LED → GND  ❌ DON'T DO THIS
PA10 (UART RX) → LED → GND  ❌ DON'T DO THIS

Updated (CORRECT):
PC13 → [220Ω Resistor] → LED (RX Activity) → GND  ✅
PC14 → [220Ω Resistor] → LED (TX Activity) → GND  ✅

PA9  → UART TX (no LED)  ✅
PA10 → UART RX (no LED)  ✅
```

### Wiring Diagram

```
STM32F401RC

PC13 ──┬──[220Ω]──┬──[LED]──┬─── GND
       │          │         │
      Pin      Resistor   Anode
                          Cathode

PC14 ──┬──[220Ω]──┬──[LED]──┬─── GND
       │          │         │
      Pin      Resistor   Anode
                          Cathode

Note: Use 220Ω resistor for red/green LEDs
      Use 100Ω resistor for blue/white LEDs
```

### Physical Connections

```
STM32F401RC (LQFP64 Package)

Pin 2:  PC13 → [220Ω] → LED (Green) → GND  (RX Activity)
Pin 3:  PC14 → [220Ω] → LED (Red)   → GND  (TX Activity)
Pin 18: GND  → Common Ground

LED Polarity:
  Long leg (+) → Resistor → STM32 Pin
  Short leg (-) → GND
```

---

## 🔧 Alternative Pin Options

If PC13/PC14 are already used, choose from these available pins:

### Option 1: Use PA4-PA7 (ADC pins, if not needed)
```c
#define LED_RX_PIN    GPIO_PIN_5   // PA5
#define LED_TX_PIN    GPIO_PIN_6   // PA6
#define LED_PORT      GPIOA
```

### Option 2: Use PB Pins
```c
#define LED_RX_PIN    GPIO_PIN_10  // PB10
#define LED_TX_PIN    GPIO_PIN_12  // PB12
#define LED_PORT      GPIOB
```

### Option 3: Use PC Pins
```c
#define LED_RX_PIN    GPIO_PIN_13  // PC13 (built-in LED on many boards)
#define LED_TX_PIN    GPIO_PIN_14  // PC14
#define LED_PORT      GPIOC
```

---

## 📊 LED Behavior

With the updated code, LEDs will:

| LED | Pin | Behavior | Meaning |
|-----|-----|----------|---------|
| Green | PC13 | **Toggles on RX** | Data received from Raspberry Pi |
| Red | PC14 | **Toggles on TX** | Data transmitted (less visible) |

### Expected Visual Behavior

```
Idle State:
  Green LED: OFF
  Red LED:   OFF

Receiving 'F' command:
  Green LED: Blinks once ✓
  Red LED:   May blink briefly

Continuous commands:
  Green LED: Flickers rapidly ✓✓✓
  Red LED:   May flicker

With Heartbeat (every 1.5s from Pi):
  Green LED: Blinks regularly ✓...✓...✓
```

---

## 🛠️ Current Limiting Resistor Selection

### Calculation Formula

```
R = (Vcc - Vf) / I_desired

Where:
  Vcc = Supply voltage (3.3V)
  Vf = LED forward voltage (depends on color)
  I_desired = Desired LED current (typically 5-15mA)
```

### Common LED Types

| LED Color | Vf (typical) | R for 10mA | R for 5mA | Standard Value |
|-----------|--------------|------------|-----------|----------------|
| Red       | 2.0V         | 130Ω       | 260Ω      | **220Ω** ✓     |
| Green     | 2.1V         | 120Ω       | 240Ω      | **220Ω** ✓     |
| Yellow    | 2.0V         | 130Ω       | 260Ω      | **220Ω** ✓     |
| Blue      | 3.0V         | 30Ω        | 60Ω       | **100Ω** ✓     |
| White     | 3.2V         | 10Ω        | 20Ω       | **47Ω** ✓      |

**Recommendation**: Use **220Ω** for most LEDs. It's safe and provides good brightness.

---

## 🔍 Troubleshooting Original Setup

### If You Must Keep LEDs on PA9/PA10 (Not Recommended)

**Add Current Limiting Resistors:**

```
BEFORE (Dangerous):
PA9  ──┬──[LED]──┬─── GND  ❌
       │         │
     3.3V    No protection

AFTER (Safer):
PA9  ──┬──[470Ω]──┬──[LED]──┬─── GND  ⚠️
       │          │         │
     3.3V    Resistor   Protected

Use 470Ω or higher to reduce current below 5mA
```

**Why PA10 Appears Brighter:**

1. **Pull-up resistor** keeps it HIGH continuously
2. **Input mode** draws minimal current
3. **Not toggling** like TX pin
4. LED effectively has:
   ```
   3.3V ─┬─[40kΩ Pull-up]─┬─ PA10
         │                │
       [LED]          [UART Logic]
         │
        GND
   ```
5. Very low current (~0.5mA) through LED
6. Just enough to glow dimly

### Why This Still Isn't Ideal

- **Signal degradation**: LED adds capacitance
- **Voltage drop**: Affects logic levels
- **Current drain**: Stresses GPIO driver
- **Unreliable**: May work, may not

---

## 🎨 Advanced: Multi-Color Status LED

For better status indication, use RGB LED:

```c
#define LED_RED_PIN      GPIO_PIN_13  // PC13
#define LED_GREEN_PIN    GPIO_PIN_14  // PC14
#define LED_BLUE_PIN     GPIO_PIN_15  // PC15
#define LED_PORT         GPIOC

// Status colors:
void Status_OK(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
}

void Status_Error(void) {
    HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_PORT, LED_BLUE_PIN, GPIO_PIN_RESET);
}

void Status_RX_Activity(void) {
    HAL_GPIO_TogglePin(LED_PORT, LED_BLUE_PIN);
}
```

---

## 📋 Hardware Checklist

Before connecting LEDs:

- [ ] **Resistor added**: 220Ω minimum (330Ω safer)
- [ ] **LED polarity correct**: Long leg (+) to resistor
- [ ] **Not on UART pins**: Use PC13/PC14 or other GPIOs
- [ ] **Current calculated**: I = (3.3V - Vf) / R < 12mA
- [ ] **Tested without LEDs**: UART works first

---

## 💡 Best Practice Recommendations

### 1. **Always Use Current Limiting Resistors**
```
Never: GPIO → LED → GND
Always: GPIO → Resistor → LED → GND
```

### 2. **Use Dedicated GPIO Pins**
```
UART Pins (PA9/PA10): Signal only
LED Pins (PC13/PC14): Indicators only
```

### 3. **Standard Resistor Values**
```
Safe for all colors: 470Ω (dim but safe)
Good brightness: 220Ω (recommended)
Maximum: 100Ω (only for high-Vf LEDs)
```

### 4. **Test Current Draw**
```c
// In code, verify:
// I = (3.3V - Vf) / R
// Example: (3.3V - 2.0V) / 220Ω = 5.9mA ✓ (safe)
```

---

## 🔬 Measuring Actual Current

To verify your setup:

```bash
# Use multimeter in series:

1. Disconnect LED from GND
2. Set multimeter to DC mA range
3. Connect: LED cathode → Multimeter (+) → Multimeter (-) → GND
4. Measure current:
   - Should be: 5-12mA ✓
   - If >20mA: Add larger resistor ⚠️
   - If <1mA: LED barely visible
```

---

## 📖 Updated Code Summary

The code has been updated with:

1. ✅ **Dedicated LED pins** (PC13/PC14)
2. ✅ **LED initialization** in GPIO_Init()
3. ✅ **Toggle on RX** in USART1_IRQHandler()
4. ✅ **Toggle on TX** (optional, can be removed if too frequent)
5. ✅ **No modification** to UART signal pins

### New Connections Required

```
Old Setup (Remove):
PA9  → LED → GND  ❌ Remove this
PA10 → LED → GND  ❌ Remove this

New Setup (Add):
PC13 → [220Ω] → Green LED → GND  ✅
PC14 → [220Ω] → Red LED → GND    ✅
```

---

## 🆘 If You Don't Have PC13/PC14

Edit `main.h` to use available pins:

```c
// Option 1: Use PA5/PA6 (if not using SPI)
#define LED_RX_PIN    GPIO_PIN_5
#define LED_TX_PIN    GPIO_PIN_6
#define LED_PORT      GPIOA

// Option 2: Use PB12/PB13
#define LED_RX_PIN    GPIO_PIN_12
#define LED_TX_PIN    GPIO_PIN_13
#define LED_PORT      GPIOB

// Don't forget to enable the clock in GPIO_Init():
__HAL_RCC_GPIOB_CLK_ENABLE();  // If using GPIOB
```

---

## 📚 Related Documentation

- **STM32F401 Datasheet**: Section 5.3.17 (I/O port characteristics)
- **GPIO Application Note**: AN4899 (Hardware) 
- **LED Drive Circuits**: See electronics tutorials

---

**Bottom Line**: 
1. ❌ Don't connect LEDs to UART pins without resistors
2. ✅ Use dedicated GPIO pins (PC13/PC14)
3. ✅ Always use current limiting resistors (220Ω minimum)
4. ✅ Verify with multimeter (5-12mA is safe)

**The updated code now uses proper LED indicators! Rebuild and flash your STM32.** 🎉
