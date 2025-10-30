#!/bin/bash
#
# Quick Setup Script for Raspberry Pi Motor Controller
# Run with: bash setup.sh
#

set -e  # Exit on error

echo "=========================================="
echo "  Raspberry Pi Motor Controller Setup"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    echo "⚠️  Warning: Not running on Raspberry Pi!"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Check Python version
echo "Checking Python version..."
python_version=$(python3 --version | grep -oP '\d+\.\d+')
if (( $(echo "$python_version < 3.9" | bc -l) )); then
    echo "❌ Python 3.9+ required, found: $python_version"
    exit 1
fi
echo "✓ Python $python_version"

# Create virtual environment
echo ""
echo "Creating virtual environment..."
if [ -d "venv" ]; then
    echo "⚠️  Virtual environment already exists"
    read -p "Recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf venv
        python3 -m venv venv
    fi
else
    python3 -m venv venv
fi
echo "✓ Virtual environment created"

# Activate virtual environment
echo ""
echo "Activating virtual environment..."
source venv/bin/activate
echo "✓ Virtual environment activated"

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip -q
echo "✓ pip upgraded"

# Install requirements
echo ""
echo "Installing Python packages..."
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt -q
    echo "✓ Packages installed"
else
    echo "Installing packages manually..."
    pip install pyserial-asyncio RPi.GPIO aioconsole -q
    echo "✓ Packages installed"
fi

# Check UART configuration
echo ""
echo "Checking UART configuration..."
if grep -q "enable_uart=1" /boot/config.txt 2>/dev/null; then
    echo "✓ UART enabled in config.txt"
else
    echo "⚠️  UART not enabled in /boot/config.txt"
    echo ""
    echo "To enable UART, add these lines to /boot/config.txt:"
    echo "  enable_uart=1"
    echo "  dtoverlay=disable-bt"
    echo ""
    echo "Then reboot with: sudo reboot"
fi

# Check serial device
echo ""
echo "Checking serial device..."
if [ -e "/dev/serial0" ]; then
    echo "✓ /dev/serial0 exists"
    ls -l /dev/serial0
else
    echo "⚠️  /dev/serial0 not found"
    echo "Available serial devices:"
    ls -l /dev/tty* 2>/dev/null | grep -E "(ttyS|ttyAMA|ttyUSB)" || echo "None found"
fi

# Check permissions
echo ""
echo "Checking user permissions..."
if groups $USER | grep -q "dialout"; then
    echo "✓ User in dialout group"
else
    echo "⚠️  User not in dialout group"
    echo "Run: sudo usermod -a -G dialout $USER"
    echo "Then logout and login again"
fi

if groups $USER | grep -q "gpio"; then
    echo "✓ User in gpio group"
else
    echo "⚠️  User not in gpio group"
    echo "Run: sudo usermod -a -G gpio $USER"
    echo "Then logout and login again"
fi

# Test import
echo ""
echo "Testing Python imports..."
if python3 -c "import serial_asyncio" 2>/dev/null; then
    echo "✓ serial_asyncio"
else
    echo "❌ serial_asyncio import failed"
fi

if python3 -c "import RPi.GPIO" 2>/dev/null; then
    echo "✓ RPi.GPIO"
else
    echo "⚠️  RPi.GPIO import failed (OK if not on Pi)"
fi

if python3 -c "import aioconsole" 2>/dev/null; then
    echo "✓ aioconsole"
else
    echo "⚠️  aioconsole import failed"
fi

# Summary
echo ""
echo "=========================================="
echo "  Setup Complete!"
echo "=========================================="
echo ""
echo "To run the motor controller:"
echo "  1. Activate virtual environment:"
echo "     source venv/bin/activate"
echo ""
echo "  2. Run the program:"
echo "     python3 rpi_motor_controller.py"
echo ""
echo "  3. Or run examples:"
echo "     python3 simple_examples.py"
echo ""
echo "For first-time setup, remember to:"
echo "  - Enable UART in /boot/config.txt"
echo "  - Add user to dialout and gpio groups"
echo "  - Disable serial console (raspi-config)"
echo "  - Connect hardware (see RPI_SETUP_GUIDE.md)"
echo ""
echo "Documentation:"
echo "  - RPI_SETUP_GUIDE.md      (Detailed setup)"
echo "  - MOTOR_CONTROLLER_README.md  (Full docs)"
echo "  - QUICK_REFERENCE.md      (Quick ref)"
echo ""
