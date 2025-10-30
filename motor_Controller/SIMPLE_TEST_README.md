# Simple Motor Control - Wiring Guide

## Pin Configuration

### Output Pins (To MX1508)
```
STM32F401    →    MX1508
─────────────────────────
PA0          →    IN1 (Motor 1 Forward)
PA1          →    IN2 (Motor 1 Reverse)
PA2          →    IN3 (Motor 2 Forward)
PA3          →    IN4 (Motor 2 Reverse)
GND          →    GND
```

### Input Pin (Switch)
```
STM32F401    ←    Switch
─────────────────────────
PA6          ←    Switch (Pull-down configured)
             →    GND (other side)
```

## How It Works

1. **Switch is HIGH (PA6 = 1):**
   - PA0 = HIGH (Motor 1 forward)
   - PA1 = LOW
   - PA2 = HIGH (Motor 2 forward)
   - PA3 = LOW
   - **Result: Both motors move forward**

2. **Switch is LOW (PA6 = 0):**
   - PA0 = LOW (Motor 1 stop)
   - PA1 = LOW
   - PA2 = LOW (Motor 2 stop)
   - PA3 = LOW
   - **Result: Both motors stop**

## MX1508 Motor Control Logic (per motor)

| IN1 (Fwd) | IN2 (Rev) | Result |
|-----------|-----------|--------|
| 1 | 0 | FORWARD |
| 0 | 1 | REVERSE |
| 0 | 0 | STOP |

## Testing

1. Compile and flash the code
2. Connect power to STM32 and MX1508
3. Set PA6 HIGH → Motors should move forward
4. Set PA6 LOW → Motors should stop
5. Verify both motors spin in the same direction

## Next Steps

Once this works:
- Add reverse motion (set PA1=1, PA0=0 when another switch pressed)
- Add left/right turn control
- Add PWM for speed control
