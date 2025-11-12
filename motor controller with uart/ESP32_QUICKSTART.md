wi# ESP32 Navigation Integration - Quick Start Guide

## 🎯 Goal
Connect ESP32 (navigator) → Raspberry Pi (orchestrator) → STM32 (motor controller)

---

## 📋 Prerequisites

### Hardware Required
- [x] ESP32 DevKit (any model)
- [x] Raspberry Pi 4B (already have)
- [x] STM32F401RC (already have)
- [ ] USB-Serial adapter (if using Serial mode) **OR** WiFi router (if using WiFi mode)

### Software Required
- [ ] Arduino IDE or PlatformIO
- [ ] ArduinoJson library (ESP32)
- [ ] Python 3.9+ (RPi)
- [ ] pyserial-asyncio (RPi)

---

## 🚀 Step-by-Step Setup

### **Step 1: Choose Communication Method**

#### **Option A: WiFi (Recommended for testing)**
✅ **Pros:** Wireless, easy debugging, no extra hardware  
❌ **Cons:** Requires WiFi network, slight latency

#### **Option B: Serial UART**
✅ **Pros:** Low latency, reliable, no WiFi needed  
❌ **Cons:** Requires USB-Serial adapter, more wiring

**Decision:** Start with WiFi for easy testing, switch to Serial if needed.

---

### **Step 2: Set Up ESP32**

#### 2.1 Install Arduino IDE
```bash
# Download from: https://www.arduino.cc/en/software
# Install ESP32 board support:
# File → Preferences → Additional Board Manager URLs:
https://dl.espressif.com/dl/package_esp32_index.json

# Tools → Board Manager → Search "ESP32" → Install
```

#### 2.2 Install ArduinoJson Library
```bash
# In Arduino IDE:
# Tools → Manage Libraries
# Search: "ArduinoJson"
# Install version 6.x
```

#### 2.3 Configure WiFi Settings
Open `esp32_navigator.ino` and edit:
```cpp
const char* WIFI_SSID = "YourWiFiName";      // Your WiFi name
const char* WIFI_PASSWORD = "YourPassword";   // Your WiFi password
const int SERVER_PORT = 8888;                 // Keep as 8888
```

#### 2.4 Upload to ESP32
```bash
# Connect ESP32 to PC via USB
# Select: Tools → Board → ESP32 Dev Module
# Select: Tools → Port → (your COM port)
# Click: Upload button
```

#### 2.5 Get ESP32 IP Address
```bash
# Open Serial Monitor (115200 baud)
# Look for: "IP Address: 192.168.1.XXX"
# Write down this IP address!
```

---

### **Step 3: Set Up Raspberry Pi**

#### 3.1 Install Python Dependencies
```bash
# SSH to Raspberry Pi
ssh pi@raspberrypi.local

# Create virtual environment
cd ~/robot_project
python3 -m venv venv
source venv/bin/activate

# Install packages
pip install pyserial-asyncio
```

#### 3.2 Copy Files to RPi
```bash
# From your Windows machine, copy files to RPi:
scp navigation_interface.py pi@raspberrypi.local:~/robot_project/
scp rpi_motor_controller.py pi@raspberrypi.local:~/robot_project/

# Or use SFTP/FileZilla
```

#### 3.3 Test Navigation Interface
```bash
# On Raspberry Pi:
cd ~/robot_project
source venv/bin/activate

# Edit navigation_interface.py to set ESP32 IP:
nano navigation_interface.py
# Change: wifi_host="192.168.1.XXX"  (your ESP32 IP)

# Run test:
python3 navigation_interface.py
```

**Expected Output:**
```
Navigation Interface Test
Connecting to ESP32...
✓ Connected to ESP32 via WiFi
Connected! Waiting for commands...
```

---

### **Step 4: Test Complete Chain**

#### 4.1 Test ESP32 → RPi Communication
```bash
# Terminal 1 (Raspberry Pi):
python3 navigation_interface.py

# You should see commands from ESP32:
📍 Navigation: TURN_LEFT (angle=90°, dist=0m, speed=60%)
📍 Navigation: MOVE_FORWARD (angle=0°, dist=2.0m, speed=70%)
```

#### 4.2 Test RPi → STM32 Communication
```bash
# Make sure STM32 is connected to RPi GPIO 14/15
# Make sure STM32 firmware is flashed

# Terminal 1 (Raspberry Pi):
python3 rpi_motor_controller.py

# Select mode: 1 (Interactive)
# Press: W (forward), S (stop), A (left), D (right)

# You should see:
▲ Moving forward
📩 STM32: US A=25cm B=30cm  (if ultrasonic enabled)
```

---

### **Step 5: Full Integration Test**

Now we'll create a simple integration script that ties everything together:

#### 5.1 Create Integration Script
```python
# File: test_integration.py
# (Will be provided in next step)
```

#### 5.2 Run Full Test
```bash
# Raspberry Pi:
python3 test_integration.py

# Expected behavior:
# 1. ESP32 sends "TURN_LEFT 90°"
# 2. RPi receives command
# 3. RPi sends 'L' to STM32
# 4. STM32 turns robot left
# 5. RPi sends status back to ESP32
```

---

## 📊 Testing Checklist

### Test 1: ESP32 WiFi Connection
- [ ] ESP32 connects to WiFi
- [ ] Serial monitor shows IP address
- [ ] Can ping ESP32 from RPi: `ping 192.168.1.XXX`

### Test 2: ESP32 → RPi Communication
- [ ] RPi receives JSON messages from ESP32
- [ ] Messages are properly parsed
- [ ] All commands received correctly

### Test 3: RPi → ESP32 Feedback
- [ ] RPi sends status messages
- [ ] ESP32 receives and parses status
- [ ] Position/heading updated

### Test 4: RPi → STM32 Commands
- [ ] RPi sends 'F' → STM32 moves forward
- [ ] RPi sends 'L' → STM32 turns left
- [ ] RPi sends 'T' → STM32 turns right
- [ ] RPi sends 'S' → STM32 stops

### Test 5: Full Chain Integration
- [ ] ESP32 sends navigation command
- [ ] RPi translates to motor command
- [ ] STM32 executes movement
- [ ] Ultrasonic feedback works
- [ ] Status sent back to ESP32

---

## 🔧 Troubleshooting

### Problem: ESP32 won't connect to WiFi
```cpp
// Solution 1: Check WiFi credentials
// Solution 2: Check WiFi band (ESP32 only supports 2.4GHz)
// Solution 3: Check Serial Monitor for error messages
```

### Problem: RPi can't connect to ESP32
```bash
# Check if ESP32 is reachable:
ping 192.168.1.XXX

# Check if port is open:
nc -zv 192.168.1.XXX 8888

# Check firewall (if using router):
# Make sure port 8888 is not blocked
```

### Problem: No communication between RPi and STM32
```bash
# Check UART connection:
ls -l /dev/serial0
# Should show: /dev/serial0 -> ttyAMA0

# Enable hardware UART:
sudo raspi-config
# Interface Options → Serial Port
# Login shell: NO
# Serial hardware: YES
# Reboot: sudo reboot

# Test UART loopback:
# Connect GPIO 14 to GPIO 15 (TX to RX)
echo "test" > /dev/serial0
cat /dev/serial0
```

### Problem: Commands received but robot doesn't move
```bash
# Check STM32 firmware:
# - Make sure firmware is flashed
# - Check motor driver connections
# - Verify power supply to motors

# Check command format:
# RPi should send single bytes: b'F', b'L', etc.
# Not strings like "FORWARD"
```

---

## 📝 Next Steps

Once basic communication works:

1. **Calibrate Turn Angles**
   - Measure how long to turn for 90°
   - Update timing in navigation executor

2. **Calibrate Forward Distance**
   - Measure how long to move for 1 meter
   - Update timing in navigation executor

3. **Implement Command Arbitrator**
   - Priority queue for commands
   - Obstacle override logic

4. **Add YOLO Integration**
   - Dynamic obstacle detection
   - Traffic sign recognition

---

## 🎥 Demo Sequence

Once everything works, try this demo:

```python
# Demo: Square Pattern
# ESP32 sends:
1. MOVE_FORWARD, distance=1.0m
2. TURN_RIGHT, angle=90°
3. MOVE_FORWARD, distance=1.0m
4. TURN_RIGHT, angle=90°
5. MOVE_FORWARD, distance=1.0m
6. TURN_RIGHT, angle=90°
7. MOVE_FORWARD, distance=1.0m
8. STOP

# Robot should move in a square pattern!
```

---

## 📞 Need Help?

Common issues and solutions in `IMPLEMENTATION_PLAN.md`

**Debug Output Levels:**
- ESP32: Serial Monitor @ 115200 baud
- RPi: Terminal with `python3 -u` (unbuffered output)
- STM32: UART debug via RPi (if enabled)

---

## ✅ Success Criteria

You've successfully completed Step 1 when:
- [x] ESP32 connects to WiFi and starts TCP server
- [x] RPi connects to ESP32 and receives commands
- [x] RPi translates commands and sends to STM32
- [x] STM32 executes motor movements
- [x] Full command loop works: ESP32 → RPi → STM32 → feedback → ESP32

**Time estimate:** 2-4 hours for first-time setup

---

**Ready to start? Begin with Step 1: Choose Communication Method!**
