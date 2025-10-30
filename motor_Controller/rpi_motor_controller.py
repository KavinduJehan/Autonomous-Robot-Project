#!/usr/bin/env python3
"""
Raspberry Pi Motor Controller for STM32F401
- Sends commands via UART to control 2 DC motors
- Commands: F=Forward, B=Backward, L=Left, R=Right, S=Stop
- UART: 115200 baud, 8N1
- Pins: GPIO14 (TX) → STM32 PA10 (RX)
        GPIO15 (RX) → STM32 PA9  (TX)
"""

import serial
import time
import sys

class MotorController:
    def __init__(self, port: str = '/dev/ttyAMA0', baud: int = 115200):
        """Initialize UART connection to STM32."""
        try:
            print(f"Attempting to connect to {port} at {baud} baud...")
            
            self.uart = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0  # 1 second timeout
            )
            
            # Verify port is actually open
            if not self.uart.is_open:
                print("Error: Port didn't open!")
                sys.exit(1)
                
            print("Port opened successfully")
            
            # Clear any startup garbage
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            print("Buffers cleared")
            
            # Wait for STM32 boot delay to finish
            print("Waiting for STM32 boot (3 seconds)...")
            time.sleep(3)  # Wait 3 seconds for STM32 to initialize
            
            # Test communication
            print("Testing communication...")
            self.uart.write(b'S')  # Send stop command
            self.uart.flush()
            time.sleep(0.1)
            
            if self.uart.in_waiting:
                response = self.uart.read(self.uart.in_waiting)
                print(f"Initial response: {[hex(b) for b in response]}")
            
            print(f"✓ Connected to STM32F401 on {port} at {baud} baud")
            print(f"DTR={self.uart.dtr}, RTS={self.uart.rts}, CTS={self.uart.cts}, DSR={self.uart.dsr}")
            
        except serial.SerialException as e:
            print(f"Error: Could not open {port}!")
            print("1. Check if UART is enabled in raspi-config")
            print("2. Verify TX/RX connections are correct")
            print(f"Error details: {str(e)}")
            sys.exit(1)
    
    def send_command(self, cmd: str) -> bool:
        """Send a single character command to STM32."""
        # Remove any whitespace, newlines, etc
        cmd = cmd.strip()
        
        # Check length after stripping
        if not cmd or len(cmd) != 1:
            print("Error: Command must be a single character")
            return False
            
        # Convert to uppercase and validate
        cmd = cmd.upper()
        if cmd not in ['F', 'B', 'L', 'R', 'S']:
            print(f"Error: '{cmd}' is not a valid command")
            return False
            
        try:
            # Debug: Print exact byte being sent
            byte_to_send = cmd.encode('ascii')
            print(f"Sending byte: {byte_to_send[0]:02X}h ({len(byte_to_send)} bytes)")
            
            # Send just the single byte, no newline
            bytes_written = self.uart.write(byte_to_send)
            self.uart.flush()  # Make sure it's sent immediately
            
            # Verify bytes written
            if bytes_written != 1:
                print(f"Error: Wrote {bytes_written} bytes instead of 1")
                return False
                
            # Try to read back any response (for debugging)
            time.sleep(0.1)  # Give STM32 time to respond
            if self.uart.in_waiting:
                response = self.uart.read(self.uart.in_waiting)
                print(f"Got response: {[hex(b) for b in response]}")
            
            return True
            
        except Exception as e:
            print(f"Send error: {e}")
            return False
    
    def close(self):
        """Close UART connection safely."""
        if hasattr(self, 'uart') and self.uart.is_open:
            self.uart.close()


def main():
    """Interactive motor control loop."""
    # Try both possible serial devices
    try:
        controller = MotorController('/dev/ttyAMA0')
    except:
        try:
            controller = MotorController('/dev/serial0')
        except:
            print("Could not open either /dev/ttyAMA0 or /dev/serial0")
            print("\nPlease follow these steps to enable UART:")
            print("1. sudo nano /boot/firmware/config.txt")
            print("   Add these lines at the end:")
            print("   enable_uart=1")
            print("   dtoverlay=disable-bt")
            print("\n2. sudo nano /boot/firmware/cmdline.txt")
            print("   Make sure there's NO console=serial0 line")
            print("   Should look like:")
            print("   console=tty1 root=PARTUUID=... rootfstype=ext4 fsck.repair=yes rootwait")
            print("\n3. sudo reboot")
            print("\n4. After reboot, verify UART is enabled:")
            print("   ls -l /dev/ttyAMA0")
            print("   sudo chmod 666 /dev/ttyAMA0")
            print("\n5. Try again:")
            print("   python3 rpi_motor_controller.py")
            sys.exit(1)
    
    print("Motor Controller Ready!")
    print("Commands: F=Forward, B=Backward, L=Left, R=Right, S=Stop, Q=Quit")
    
    try:
        while True:
            # Get input and clean it thoroughly
            cmd = input("Enter command: ").strip()
            
            # Debug: show exact bytes that would be sent
            if cmd:
                print(f"Command byte value: {[hex(b) for b in cmd.encode('ascii')]}")
            
            if cmd.upper() == 'Q':
                break
                
            if controller.send_command(cmd):
                print(f"Sent: '{cmd.upper()}'")  # Show normalized command
            else:
                print("Invalid command! Use F, B, L, R, or S")
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Always stop motors before exit
        controller.send_command('S')
        controller.close()


if __name__ == "__main__":
    main()
