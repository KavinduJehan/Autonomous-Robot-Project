"""
UART Motor Controller Test Script
==================================
This script tests the STM32F401RC motor controller via UART.

Requirements:
    pip install pyserial

Usage:
    python uart_test.py COM3  # Windows
    python uart_test.py /dev/ttyUSB0  # Linux
    python uart_test.py /dev/cu.usbserial-xxx  # macOS

Commands:
    F - Forward
    R - Reverse
    L - Left turn
    T - Right turn
    S - Stop
    Q - Quit
"""

import serial
import time
import sys

# Configuration
BAUD_RATE = 9600
TIMEOUT = 1.0

# Command definitions
COMMANDS = {
    'F': 'Forward - Both motors forward',
    'R': 'Reverse - Both motors backward',
    'L': 'Left - Spot turn left',
    'T': 'Right - Spot turn right',
    'S': 'Stop - All motors off',
    'Q': 'Quit program'
}

def print_menu():
    """Print command menu."""
    print("\n" + "="*50)
    print("STM32 Motor Controller - UART Test")
    print("="*50)
    for cmd, desc in COMMANDS.items():
        print(f"  {cmd} - {desc}")
    print("="*50)

def send_command(ser, cmd):
    """Send command to STM32 and display status."""
    try:
        ser.write(cmd.encode())
        print(f"✓ Sent: '{cmd}' - {COMMANDS.get(cmd, 'Unknown command')}")
        time.sleep(0.1)  # Small delay for processing
        
        # Check if there's any response (for loopback testing)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"  Response: {response}")
            
    except Exception as e:
        print(f"✗ Error sending command: {e}")

def automated_test(ser):
    """Run automated test sequence."""
    print("\n🤖 Running Automated Test Sequence...")
    print("-" * 50)
    
    test_sequence = [
        ('S', 1),   # Stop - ensure motors off
        ('F', 2),   # Forward for 2 seconds
        ('S', 1),   # Stop
        ('R', 2),   # Reverse for 2 seconds
        ('S', 1),   # Stop
        ('L', 2),   # Left turn for 2 seconds
        ('S', 1),   # Stop
        ('T', 2),   # Right turn for 2 seconds
        ('S', 1),   # Stop
    ]
    
    for cmd, duration in test_sequence:
        print(f"\nTest: {cmd} - {COMMANDS[cmd]} ({duration}s)")
        send_command(ser, cmd)
        time.sleep(duration)
    
    print("\n✓ Automated test complete!")
    print("\n⚠️  Testing safety timeout (2 seconds)...")
    print("    Motors should stop automatically after 2 seconds...")
    time.sleep(2.5)
    print("✓ Timeout test complete")

def manual_control(ser):
    """Manual control mode."""
    print("\n🎮 Manual Control Mode")
    print("   Type commands (F/R/L/T/S) or Q to quit")
    
    while True:
        cmd = input("\nCommand > ").strip().upper()
        
        if not cmd:
            continue
            
        if cmd == 'Q':
            # Send stop before quitting
            send_command(ser, 'S')
            print("👋 Exiting...")
            break
            
        if cmd in ['F', 'R', 'L', 'T', 'S']:
            send_command(ser, cmd)
        else:
            print(f"✗ Invalid command: '{cmd}'")
            print("   Valid commands: F, R, L, T, S, Q")

def test_connection(ser):
    """Test UART connection."""
    print("\n🔍 Testing UART Connection...")
    print(f"   Port: {ser.port}")
    print(f"   Baud: {ser.baudrate}")
    print(f"   Timeout: {ser.timeout}s")
    
    # Send stop command as connection test
    try:
        ser.write(b'S')
        print("✓ Connection OK - Stop command sent")
        time.sleep(0.5)
        return True
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return False

def main():
    """Main function."""
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Usage: python uart_test.py <COM_PORT>")
        print("\nExamples:")
        print("  Windows: python uart_test.py COM3")
        print("  Linux:   python uart_test.py /dev/ttyUSB0")
        print("  macOS:   python uart_test.py /dev/cu.usbserial-xxx")
        sys.exit(1)
    
    port = sys.argv[1]
    
    # Initialize serial connection
    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )
        
        print(f"✓ Connected to {port} at {BAUD_RATE} baud")
        
    except serial.SerialException as e:
        print(f"✗ Failed to open port {port}: {e}")
        print("\nAvailable ports:")
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(f"  - {p.device}: {p.description}")
        sys.exit(1)
    
    try:
        # Test connection
        if not test_connection(ser):
            return
        
        # Print menu
        print_menu()
        
        # Ask for test mode
        print("\n📋 Select Mode:")
        print("  1 - Manual Control")
        print("  2 - Automated Test")
        
        mode = input("\nMode > ").strip()
        
        if mode == '1':
            manual_control(ser)
        elif mode == '2':
            automated_test(ser)
            # Offer manual control after automated test
            if input("\nSwitch to manual control? (y/n) > ").lower() == 'y':
                manual_control(ser)
        else:
            print("Invalid mode selected")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        send_command(ser, 'S')  # Emergency stop
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        
    finally:
        # Always send stop before closing
        try:
            ser.write(b'S')
            print("✓ Emergency stop sent")
        except:
            pass
        
        ser.close()
        print("✓ Serial port closed")

if __name__ == "__main__":
    main()
