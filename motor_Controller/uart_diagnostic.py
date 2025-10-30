#!/usr/bin/env python3
"""
UART Diagnostic Tool - Test communication with STM32
"""

import serial
import time

def test_uart():
    print("=== UART Diagnostic Tool ===\n")
    
    try:
        print("1. Opening /dev/ttyAMA0 at 115200 baud...")
        uart = serial.Serial(
            port='/dev/ttyAMA0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2.0
        )
        print("   ✓ Port opened\n")
        
        # Clear buffers
        uart.reset_input_buffer()
        uart.reset_output_buffer()
        
        print("2. Waiting 3 seconds for STM32 boot...")
        time.sleep(3)
        print("   ✓ Boot delay done\n")
        
        # Test 1: Send single byte and check if anything comes back
        print("3. Sending 'S' (STOP) command...")
        uart.write(b'S')
        uart.flush()
        print("   ✓ Sent: 0x53 (S)\n")
        
        print("4. Listening for response (2 second timeout)...")
        time.sleep(0.5)
        
        if uart.in_waiting:
            response = uart.read(uart.in_waiting)
            print(f"   ✓ Got response: {[hex(b) for b in response]}")
            print(f"   Response length: {len(response)} bytes\n")
        else:
            print("   ✗ No response from STM32\n")
        
        # Test 2: Send all motor commands and watch for activity
        commands = [
            ('F', 'Forward'),
            ('B', 'Backward'),
            ('L', 'Left'),
            ('R', 'Right'),
            ('S', 'Stop'),
        ]
        
        print("5. Testing all motor commands...\n")
        
        for cmd, name in commands:
            print(f"   Sending '{cmd}' ({name})...")
            uart.write(cmd.encode('ascii'))
            uart.flush()
            
            time.sleep(0.2)
            
            if uart.in_waiting:
                response = uart.read(uart.in_waiting)
                print(f"      Response: {[hex(b) for b in response]}")
            else:
                print(f"      No response")
            
            time.sleep(0.3)
        
        print("\n6. Checking serial port parameters:")
        print(f"   Port: {uart.port}")
        print(f"   Baud rate: {uart.baudrate}")
        print(f"   Byte size: {uart.bytesize}")
        print(f"   Parity: {uart.parity}")
        print(f"   Stop bits: {uart.stopbits}")
        print(f"   Timeout: {uart.timeout}")
        print(f"   Is open: {uart.is_open}\n")
        
        uart.close()
        
        print("=== Diagnostic Complete ===")
        print("\nNEXT STEPS:")
        print("1. Check STM32 firmware is programmed correctly")
        print("2. Verify TX/RX wires:")
        print("   - RPi GPIO14 (TX) → STM32 PA10 (RX)")
        print("   - RPi GPIO15 (RX) → STM32 PA9 (TX)")
        print("3. Check GND connection between RPi and STM32")
        print("4. If still no response, check STM32 with oscilloscope on PA9/PA10")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_uart()
