# Quick Ultrasonic Debug Guide - "U" Command Not Showing Distances

## Symptom
When you press `U` in the Python console, you see:
```
📡 Ultrasonic ping sent
   Left: 0.0cm, Right: 0.0cm
```

But you DON'T see a line like:
```
  📩 STM32: US A=25cm B=30cm
```

## Root Cause Analysis

### What should happen:
1. Python sends 'U' to STM32
2. STM32 receives 'U' in UART interrupt
3. Command processor calls `CMD_ULTRASONIC_PING` handler
4. Handler measures both sensors and prints: `US A=XXcm B=YYcm\r\n`
5. Python UART reader receives this line
6. Parser extracts distances and updates `self.ultrasonic_left_cm/right_cm`
7. `request_ultrasonic_ping()` prints the stored values

### Possible failures:

#### Issue 1: STM32 not receiving 'U' command
**Check:**
- Run Python script and press 'U'
- If you see other UART output (like "US L=XXcm R=YYcm" every 500ms from wall_avoidance task), then UART RX/TX are working
- If you see NOTHING at all from STM32, check:
  - TX/RX swap (STM32 PA9→RPI RX, STM32 PA10←RPI TX)
  - Ground connection
  - Baud rate mismatch (both should be 9600)

#### Issue 2: 'U' command not reaching command processor
**Check:**
- In `command_processor.c`, the 'U' command (CMD_ULTRASONIC_PING) is handled
- The UART interrupt in `uart_comm.c` should queue the command
- Motor task should dequeue and process it

**Quick test:**
Add a debug line BEFORE the ultrasonic measurement in `command_processor.c`:
```c
case CMD_ULTRASONIC_PING:
{
    UART_SendString("DBG: U cmd received\r\n");  // ADD THIS
#if ULTRASONIC_ENABLED
    uint16_t a = Ultrasonic_MeasureA();
    // ... rest of code
```

Rebuild, flash, press 'U'. If you see "DBG: U cmd received" but NOT "US A=...", then the ultrasonic measurement itself is timing out.

#### Issue 3: Ultrasonic sensors both timing out (returning 0)
**Check:**
- Look for the automatic debug output every 500ms from `wall_avoidance.c`:
  ```
  📩 STM32: US L=XXcm R=YYcm LEDL=X LEDR=X CMD=X
  ```
- If BOTH are always 0, then:
  - HC-SR04 modules not powered (need 5V, not 3.3V)
  - TRIG pins not connected (PB0, PB1)
  - ECHO pins not connected (PB6, PB7)
  - ECHO voltage dividers reversed (2k on top / 1k on bottom gives 1.67V instead of 3.3V)

#### Issue 4: Python timing issue
The `request_ultrasonic_ping()` waits 0.2s for the response, but if the UART read loop is slow or the STM32 takes longer to respond, you might print the old (0) values before the new ones arrive.

**Workaround:**
Instead of using the cached values, wait for a fresh "US A=" line after sending 'U'.

## Quick Fixes

### Fix 1: Increase Python wait time (temporary test)
In `rpi_motor_controller.py`, change:
```python
async def request_ultrasonic_ping(self):
    await self._send_command(MotorCommand.ULTRASONIC_PING)
    print("📡 Ultrasonic ping sent")
    await asyncio.sleep(0.5)  # Increase from 0.2 to 0.5
    left, right = self.get_ultrasonic_distances()
    print(f"   Left: {left:.1f}cm, Right: {right:.1f}cm")
```

### Fix 2: Check if you see ANY STM32 output in Python
Run the script and just wait without pressing anything. Every 500ms you should see:
```
  📩 STM32: US L=XXcm R=YYcm LEDL=X LEDR=X CMD=X
```

If you see this, UART is working and sensors are measuring (or timing out consistently).

If you DON'T see this:
- Python UART reader not starting
- STM32 not transmitting
- Wrong serial port in Python config

### Fix 3: Test STM32 directly without Python
Use a terminal program (PuTTY, minicom, screen) to connect to STM32:
```bash
# Windows (PowerShell)
# Check COM port in Device Manager first
mode COM3 BAUD=9600 PARITY=n DATA=8

# Or use PuTTY: COM3, 9600 baud, 8N1
```

Press 'U' key directly. You should see:
```
US A=XXcm B=YYcm
```

If you see this in a terminal but NOT in Python, the issue is in the Python script.

### Fix 4: Add explicit ACK in STM32
In `command_processor.c`, add a clear response for 'U':
```c
case CMD_ULTRASONIC_PING:
{
    UART_SendString("ACK_U\r\n");  // Immediate acknowledgment
#if ULTRASONIC_ENABLED
    uint16_t a = Ultrasonic_MeasureA();
    osDelay(5);
    uint16_t b = Ultrasonic_MeasureB();
    UART_SendString("US A="); 
    UART_SendUInt(a); 
    UART_SendString("cm B="); 
    UART_SendUInt(b); 
    UART_SendCRLF();
#else
    UART_SendString("US disabled\r\n");
#endif
    break;
}
```

Now when you press 'U', you'll immediately see "ACK_U" (proves command received), then "US A=..." (proves measurement completed).

## Diagnostic Checklist

Run through these in order:

- [ ] **Python sees automatic debug output every 500ms**
  - If YES: UART is working, proceed to next check
  - If NO: Fix UART connection (TX/RX wires, ground, baud rate)

- [ ] **Automatic debug shows non-zero distances**
  - If YES: Sensors are working, issue is with 'U' command handling
  - If NO: Fix sensor wiring/power (see ULTRASONIC_TROUBLESHOOTING.md)

- [ ] **Press 'U' in Python, see "ACK_U" or "DBG: U cmd received"**
  - If YES: Command is reaching STM32, issue is in measurement or response
  - If NO: Command not being queued/processed, check UART interrupt and queue

- [ ] **Press 'U', see "US A=XXcm B=YYcm" in Python output**
  - If YES: STM32 responding correctly, issue is Python parsing or timing
  - If NO: Measurement timing out or UART TX not working

- [ ] **Python prints distances after pressing 'U'**
  - If YES: Everything working!
  - If NO: Increase wait time or fix parser

## Common Causes (Ranked by Frequency)

1. **UART RX/TX swapped** (50%)
   - STM32 PA9 (TX) must go to RPI GPIO15 (RX)
   - STM32 PA10 (RX) must go to RPI GPIO14 (TX)

2. **Both sensors returning 0 due to wiring** (30%)
   - No 5V power to HC-SR04
   - ECHO dividers reversed
   - TRIG not connected

3. **Python timing too short** (10%)
   - 0.2s not enough for measurement + UART transmission
   - Increase to 0.5s or 1.0s

4. **Command not reaching motor task queue** (5%)
   - UART interrupt not enabled
   - Queue full or not created
   - Motor task blocked

5. **UART reader not started in Python** (5%)
   - `_uart_read_loop` not running
   - asyncio task not created
   - Serial port not opened correctly

## Expected Output (Working System)

When everything works, pressing 'U' in Python shows:

```
📡 Ultrasonic ping sent
  📩 STM32: US A=25cm B=30cm
   Left: 25.0cm, Right: 30.0cm
```

And every 500ms automatically:
```
  📩 STM32: US L=25cm R=30cm LEDL=0 LEDR=0 CMD=F
```

## Next Steps

1. Check if you see the automatic 500ms debug output
2. If NO → Fix UART wiring
3. If YES but shows 0cm → Fix sensor wiring (see ULTRASONIC_TROUBLESHOOTING.md)
4. If YES and shows real distances → Try increasing Python wait time to 0.5s
5. If still no output on 'U' → Add "ACK_U" debug line and rebuild STM32

---

**Created**: November 1, 2025  
**For issue**: Python 'U' command not showing ultrasonic distances
