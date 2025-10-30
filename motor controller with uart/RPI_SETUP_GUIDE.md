# Raspberry Pi 4B Setup Guide for Motor Controller

## 🔧 Hardware Setup

### 1. UART Connection (GPIO Pins)

The Raspberry Pi 4B has built-in UART on GPIO 14/15:

```
Raspberry Pi 4B          STM32F401RC
┌─────────────┐         ┌──────────┐
│             │         │          │
│ GPIO 14 (TX)├─────────┤ PA10 (RX)│
│             │         │          │
│ GPIO 15 (RX)├─────────┤ PA9  (TX)│
│             │         │          │
│     GND     ├─────────┤ GND      │
│             │         │          │
│ GPIO 17     ├───[BTN]─┤ GND      │ (Emergency Stop)
│             │         │          │
└─────────────┘         └──────────┘

Physical Pin Layout:
Pin 6  (GND)     ──→ STM32 GND
Pin 8  (GPIO 14) ──→ STM32 PA10 (RX)
Pin 10 (GPIO 15) ──→ STM32 PA9  (TX)
Pin 11 (GPIO 17) ──→ Emergency Stop Button → GND
```

### 2. Enable UART on Raspberry Pi

```bash
# Edit boot config
sudo nano /boot/config.txt

# Add these lines at the end:
enable_uart=1
dtoverlay=disable-bt

# Save and reboot
sudo reboot
```

### 3. Disable Serial Console (Important!)

```bash
# Disable serial console to free up UART
sudo raspi-config

# Navigate to:
# 3. Interface Options
# → P6. Serial Port
# → Login shell over serial: NO
# → Serial port hardware: YES
# Finish and reboot

# Or use command line:
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
```

---

## 📦 Software Installation

### 1. Update System

```bash
# Update package list
sudo apt update
sudo apt upgrade -y

# Install Python 3.9+ (should be already installed on Pi 4B)
python3 --version  # Should show 3.9 or higher
```

### 2. Create Project Directory

```bash
# Create project folder
mkdir -p ~/robot_controller
cd ~/robot_controller

# Download the Python script (if from GitHub)
# Or copy from your development machine
```

### 3. Create Virtual Environment

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Your prompt should now show (venv)
```

### 4. Install Dependencies

```bash
# With virtual environment activated:
pip install --upgrade pip

# Install required packages
pip install pyserial-asyncio
pip install RPi.GPIO
pip install aioconsole  # Optional: for better keyboard input

# Verify installation
pip list
```

### 5. Install Requirements File (Alternative)

Create `requirements.txt`:
```bash
cat > requirements.txt << EOF
pyserial-asyncio>=0.6
RPi.GPIO>=0.7.1
aioconsole>=0.6.0
EOF

# Install from requirements
pip install -r requirements.txt
```

---

## 🚀 Running the Program

### Quick Start

```bash
# 1. Activate virtual environment
cd ~/robot_controller
source venv/bin/activate

# 2. Run the program
python3 rpi_motor_controller.py

# 3. Select mode:
#    1 - Interactive (keyboard control)
#    2 - Demo sequence
#    3 - API mode (programmatic)
```

### Run on Startup (Optional)

Create systemd service:

```bash
# Create service file
sudo nano /etc/systemd/system/motor-controller.service
```

Add this content:
```ini
[Unit]
Description=Motor Controller Service
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/robot_controller
ExecStart=/home/pi/robot_controller/venv/bin/python3 /home/pi/robot_controller/rpi_motor_controller.py
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable motor-controller.service
sudo systemctl start motor-controller.service

# Check status
sudo systemctl status motor-controller.service

# View logs
sudo journalctl -u motor-controller.service -f
```

---

## 🧪 Testing

### 1. Check UART Connection

```bash
# Test UART loopback (disconnect from STM32 first!)
# Connect GPIO 14 to GPIO 15 with jumper wire

# In Python:
python3
>>> import serial
>>> ser = serial.Serial('/dev/serial0', 9600, timeout=1)
>>> ser.write(b'TEST')
>>> ser.read(4)  # Should return b'TEST'
>>> ser.close()
```

### 2. Test with STM32

```bash
# Connect STM32 and run:
python3 rpi_motor_controller.py

# Should see:
# ✓ Connected to motor controller

# Try commands:
# F - Forward
# S - Stop
```

### 3. Monitor Serial Communication

```bash
# Install minicom for debugging
sudo apt install minicom

# Monitor serial port
minicom -b 9600 -o -D /dev/serial0

# Press Ctrl+A then X to exit
```

---

## 🎮 Usage Examples

### Example 1: Interactive Control

```bash
python3 rpi_motor_controller.py

# Select mode: 1
# Type commands:
w  # Forward
s  # Reverse
a  # Left
d  # Right
space  # Stop
q  # Quit
```

### Example 2: Programmatic Control

Create your own script:

```python
import asyncio
from rpi_motor_controller import MotorController, ControllerConfig

async def my_robot_program():
    # Configure
    config = ControllerConfig(port='/dev/serial0', baudrate=9600)
    motor = MotorController(config)
    
    # Connect
    await motor.connect()
    
    # Start heartbeat
    heartbeat_task = asyncio.create_task(motor._heartbeat_loop())
    
    try:
        # Your robot logic here
        await motor.forward(2)       # Forward 2 seconds
        await motor.turn_right(1)    # Right turn 1 second
        await motor.forward(2)       # Forward 2 seconds
        await motor.stop()           # Stop
        
    finally:
        heartbeat_task.cancel()
        await motor.disconnect()
        motor.cleanup()

# Run
asyncio.run(my_robot_program())
```

### Example 3: Sensor Integration

```python
import asyncio
from rpi_motor_controller import MotorController, ControllerConfig
import RPi.GPIO as GPIO

# Setup ultrasonic sensor
TRIG_PIN = 23
ECHO_PIN = 24

async def obstacle_avoidance():
    motor = MotorController()
    await motor.connect()
    
    heartbeat = asyncio.create_task(motor._heartbeat_loop())
    
    try:
        while True:
            distance = measure_distance(TRIG_PIN, ECHO_PIN)
            
            if distance < 20:  # Obstacle within 20cm
                await motor.stop()
                await motor.reverse(0.5)
                await motor.turn_right(0.5)
            else:
                await motor.forward()
            
            await asyncio.sleep(0.1)
    
    finally:
        heartbeat.cancel()
        await motor.disconnect()
        motor.cleanup()

asyncio.run(obstacle_avoidance())
```

---

## 🔍 Troubleshooting

### Problem: Permission Denied on /dev/serial0

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER

# Logout and login again, or:
newgrp dialout

# Check permissions
ls -l /dev/serial0
# Should show: crw-rw---- 1 root dialout
```

### Problem: UART Not Available

**Solution:**
```bash
# Check if UART is enabled
dtoverlay -l | grep uart
# Should show: uart enabled

# Check for serial devices
ls -l /dev/serial*
ls -l /dev/ttyS*
ls -l /dev/ttyAMA*

# On Pi 4B, primary UART is:
# /dev/serial0 -> ttyAMA0 (mini UART)
# or
# /dev/serial0 -> ttyS0 (PL011)
```

### Problem: Bluetooth Conflict

**Solution:**
```bash
# Disable Bluetooth to free up UART
sudo systemctl disable hciuart
sudo systemctl disable bluetooth

# Or use mini UART for Bluetooth:
# In /boot/config.txt:
dtoverlay=miniuart-bt
enable_uart=1

sudo reboot
```

### Problem: Import Error - No module named 'serial_asyncio'

**Solution:**
```bash
# Make sure virtual environment is activated
source venv/bin/activate

# Reinstall
pip install --upgrade pyserial-asyncio

# Check installation
python3 -c "import serial_asyncio; print('OK')"
```

### Problem: GPIO Warning/Error

**Solution:**
```bash
# GPIO already in use
# Add to your script:
GPIO.setwarnings(False)

# Or cleanup before running:
python3 -c "import RPi.GPIO as GPIO; GPIO.cleanup()"
```

### Problem: No Response from STM32

**Checklist:**
- [ ] STM32 powered and programmed
- [ ] TX/RX wires not swapped (Pi TX → STM32 RX)
- [ ] Common ground connected
- [ ] Baud rate matches (9600)
- [ ] UART enabled on Pi
- [ ] Serial console disabled
- [ ] Correct device (/dev/serial0)

**Test:**
```bash
# Send test command
echo -n "S" > /dev/serial0

# Check with multimeter:
# Pi TX (GPIO 14) should toggle 0-3.3V
```

---

## 📊 Performance Tips

### 1. CPU Governor

```bash
# Set to performance mode for better response
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

### 2. Process Priority

```bash
# Run with higher priority
sudo nice -n -10 python3 rpi_motor_controller.py
```

### 3. Reduce Latency

```python
# In your code, adjust timing:
config = ControllerConfig(
    command_interval=0.05,  # Faster commands (50ms)
    heartbeat_interval=1.0   # More frequent heartbeat
)
```

---

## 🔒 Security Considerations

### 1. Disable Unnecessary Services

```bash
# List running services
systemctl list-units --type=service --state=running

# Disable unused services
sudo systemctl disable avahi-daemon
sudo systemctl disable cups
```

### 2. Firewall (if using network)

```bash
# Install UFW
sudo apt install ufw

# Allow SSH only
sudo ufw allow ssh
sudo ufw enable
```

### 3. Update Regularly

```bash
# Weekly updates
sudo apt update && sudo apt upgrade -y

# Auto-updates (optional)
sudo apt install unattended-upgrades
sudo dpkg-reconfigure --priority=low unattended-upgrades
```

---

## 📱 Remote Access

### SSH Setup

```bash
# Enable SSH
sudo raspi-config
# → Interface Options → SSH → Enable

# Connect from PC:
ssh pi@raspberrypi.local
# or
ssh pi@<IP_ADDRESS>
```

### VNC Setup (with GUI)

```bash
# Enable VNC
sudo raspi-config
# → Interface Options → VNC → Enable

# Install RealVNC Viewer on PC
# Connect to: raspberrypi.local
```

### Web Interface (Optional)

Create simple Flask web interface:

```bash
pip install flask

# Create web_controller.py with Flask routes
# Access via http://raspberrypi.local:5000
```

---

## 🎓 Learning Resources

### Python Asyncio
- Official docs: https://docs.python.org/3/library/asyncio.html
- Tutorial: Real Python Async IO

### Raspberry Pi UART
- Official docs: https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts

### GPIO Programming
- RPi.GPIO docs: https://sourceforge.net/p/raspberry-gpio-python/wiki/

---

## 📋 Quick Reference

### Common Commands

```bash
# Activate environment
source venv/bin/activate

# Deactivate environment
deactivate

# Run program
python3 rpi_motor_controller.py

# Check UART
ls -l /dev/serial0

# Monitor logs
tail -f /var/log/syslog | grep motor

# Reboot
sudo reboot
```

### File Locations

```
/home/pi/robot_controller/          # Project directory
├── venv/                            # Virtual environment
├── rpi_motor_controller.py          # Main program
├── requirements.txt                 # Dependencies
└── my_robot_program.py              # Your custom code
```

---

## 🆘 Support

For issues:
1. Check wiring connections
2. Verify UART enabled: `dtoverlay -l`
3. Check permissions: `groups $USER`
4. Test with loopback
5. Review logs: `journalctl -xe`

---

**Setup Complete! Your Raspberry Pi is ready to control the robot!** 🚀
