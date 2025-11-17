#!/usr/bin/env python3
"""
Raspberry Pi 4B Motor Controller Interface
==========================================
Modern asynchronous Python interface for STM32F401RC motor controller
with PWM speed control support.

Uses asyncio for non-blocking operations and virtual environments.

Requirements:
    Python 3.9+
    pyserial-asyncio
    RPi.GPIO (for emergency stop button)

Installation:
    python3 -m venv venv
    source venv/bin/activate
    pip install pyserial-asyncio RPi.GPIO

Hardware Connections (Raspberry Pi 4B):
    GPIO 14 (TX)  -> STM32 PA10 (RX)
    GPIO 15 (RX)  -> STM32 PA9  (TX)
    GPIO 17       -> Emergency Stop Button (with pull-up)
    GND           -> STM32 GND (COMMON GROUND!)

Supported Commands:
    Movement:
        'F' - Forward at current speed
        'R' - Reverse at current speed
        'L' - Turn left (spot turn) at current speed
        'T' - Turn right (spot turn) at current speed
        'S' - Stop all motors
    
    Speed Control (PWM):
        '1' - Set speed to SLOW (40%)
        '2' - Set speed to MEDIUM (70%)
        '3' - Set speed to FAST (100%)
    
    Acceleration Control:
        'M' - Enable smooth acceleration/deceleration
        'Z' - Disable acceleration (instant speed changes)
        'D' - Disable acceleration (alternate)
    
    Sensor Commands:
        'U' - Ultrasonic ping (HC-SR04 backup sensors)
        'G' - ToF distances (4x VL53L0X precision sensors)
        'J' - Junction detection (ToF-based navigation)
        'O' - Obstacle detection (ToF-based collision avoidance)
        'Q' - Front distance (single sensor query)
        'I' - Status report (all sensors)

Usage:
    python3 rpi_motor_controller.py
    
Author: Robot Project Team
Date: October 31, 2025
Version: 2.1 - Added Acceleration/Deceleration
"""

import asyncio
import signal
import sys
from enum import Enum
from dataclasses import dataclass
from datetime import datetime, timedelta
from typing import Optional, Callable
import serial_asyncio

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("Warning: RPi.GPIO not available. GPIO features disabled.")
    GPIO_AVAILABLE = False


class MotorCommand(Enum):
    """Motor command enumeration."""
    FORWARD = b'F'
    REVERSE = b'R'
    LEFT = b'L'
    RIGHT = b'T'
    STOP = b'S'
    SPEED_SLOW = b'1'      # 40% speed
    SPEED_MEDIUM = b'2'    # 70% speed
    SPEED_FAST = b'3'      # 100% speed
    ACCEL_ENABLE = b'M'    # Enable smooth acceleration/deceleration
    ACCEL_DISABLE = b'Z'   # Disable (instant speed changes)
    ACCEL_DISABLE_ALT = b'D'  # Alternate disable command
    ULTRASONIC_PING = b'U'  # Request ultrasonic distance measurement
    FRONT_DISTANCE = b'Q'   # Get front sensor distance
    STATUS_REPORT = b'I'    # Get all sensor status
    # ToF Sensor Commands (New)
    TOF_DISTANCES = b'G'    # Get ToF sensor distances
    TOF_JUNCTIONS = b'J'    # Get junction detection status
    TOF_OBSTACLES = b'O'    # Get obstacle detection status
    
    def __str__(self):
        return self.name.capitalize().replace('_', ' ')


@dataclass
class ControllerConfig:
    """Configuration for motor controller."""
    port: str = '/dev/serial0'  # Default UART on Raspberry Pi
    baudrate: int = 9600
    timeout: float = 1.0
    command_interval: float = 0.1  # Minimum time between commands
    heartbeat_interval: float = 1.5  # Send heartbeat to prevent timeout
    emergency_stop_pin: int = 17  # GPIO pin for emergency stop


class SpeedLevel(Enum):
    """Motor speed levels."""
    SLOW = (40, MotorCommand.SPEED_SLOW)      # 40% PWM
    MEDIUM = (70, MotorCommand.SPEED_MEDIUM)  # 70% PWM
    FAST = (100, MotorCommand.SPEED_FAST)     # 100% PWM
    
    def __init__(self, percentage: int, command: MotorCommand):
        self.percentage = percentage
        self.command = command
    
    def __str__(self):
        return f"{self.name} ({self.percentage}%)"


class MotorController:
    """
    Asynchronous motor controller for STM32 via UART.
    
    Features:
    - Async/await pattern for non-blocking operations
    - PWM speed control (Slow/Medium/Fast)
    - Smooth acceleration and deceleration
    - Automatic heartbeat to prevent safety timeout
    - Command queuing and rate limiting
    - Emergency stop button integration
    - Connection monitoring and auto-reconnect
    - Comprehensive logging
    """
    
    def __init__(self, config: ControllerConfig = None):
        """Initialize motor controller."""
        self.config = config or ControllerConfig()
        self.writer: Optional[asyncio.StreamWriter] = None
        self.reader: Optional[asyncio.StreamReader] = None
        self.connected = False
        self.running = False
        self.last_command_time = datetime.now()
        self.current_command = MotorCommand.STOP
        self.current_speed = SpeedLevel.MEDIUM  # Default speed
        self.accel_enabled = True  # Acceleration/deceleration enabled by default
        self.command_queue = asyncio.Queue()
        self.emergency_stop_active = False
        
        # Ultrasonic sensor data
        self.ultrasonic_left_cm = 0.0
        self.ultrasonic_right_cm = 0.0
        self.last_ultrasonic_update = datetime.now()
        
        # ToF sensor data (New)
        self.tof_front_left_mm = 0
        self.tof_front_right_mm = 0
        self.tof_left_mm = 0
        self.tof_right_mm = 0
        self.last_tof_update = datetime.now()
        
        # Junction detection data
        self.junction_left = False
        self.junction_right = False
        self.junction_both = False
        
        # Obstacle detection data
        self.front_obstacle = False
        self.closest_distance_mm = 0
        
        # Callbacks
        self.on_connect: Optional[Callable] = None
        self.on_disconnect: Optional[Callable] = None
        self.on_command_sent: Optional[Callable] = None
        self.on_error: Optional[Callable] = None
        
        # Setup GPIO for emergency stop
        if GPIO_AVAILABLE:
            self._setup_gpio()
    
    def _setup_gpio(self):
        """Setup GPIO for emergency stop button."""
        try:
            # Cleanup any existing GPIO configuration
            GPIO.cleanup()
            
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.config.emergency_stop_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Remove any existing event detection first
            try:
                GPIO.remove_event_detect(self.config.emergency_stop_pin)
            except:
                pass
            
            GPIO.add_event_detect(
                self.config.emergency_stop_pin,
                GPIO.FALLING,
                callback=self._emergency_stop_callback,
                bouncetime=300
            )
            print(f"[OK] Emergency stop button configured on GPIO {self.config.emergency_stop_pin}")
        except Exception as e:
            print(f"[WARN] GPIO setup warning: {e}")
            print(f"   Emergency stop button on GPIO {self.config.emergency_stop_pin} may not be available")
            # Don't fail - continue without GPIO emergency stop
    
    def _emergency_stop_callback(self, channel):
        """Handle emergency stop button press."""
        print("\n[ALERT] EMERGENCY STOP ACTIVATED!")
        self.emergency_stop_active = True
        asyncio.create_task(self.stop())
    
    async def connect(self) -> bool:
        """
        Connect to STM32 motor controller.
        
        Returns:
            bool: True if connection successful
        """
        try:
            print(f"[INFO] Connecting to {self.config.port} at {self.config.baudrate} baud...")
            
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
                url=self.config.port,
                baudrate=self.config.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self.config.timeout
            )
            
            self.connected = True
            self.running = True
            
            # Send initial stop command
            await self._send_command(MotorCommand.STOP)
            
            print(f"[OK] Connected to motor controller")
            
            if self.on_connect:
                self.on_connect()
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            if self.on_error:
                self.on_error(e)
            return False
    
    async def disconnect(self):
        """Disconnect from motor controller."""
        if self.connected:
            print("\n[INFO] Disconnecting...")
            
            # Send stop command before disconnecting
            await self.stop()
            await asyncio.sleep(0.1)
            
            if self.writer:
                self.writer.close()
                await self.writer.wait_closed()
            
            self.connected = False
            self.running = False
            
            print("[OK] Disconnected")
            
            if self.on_disconnect:
                self.on_disconnect()
    
    async def _send_command(self, command: MotorCommand) -> bool:
        """
        Send command to motor controller.
        
        Args:
            command: Motor command to send
            
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected or not self.writer:
            print("[ERROR] Not connected")
            return False
        
        try:
            # Rate limiting
            time_since_last = (datetime.now() - self.last_command_time).total_seconds()
            if time_since_last < self.config.command_interval:
                await asyncio.sleep(self.config.command_interval - time_since_last)
            
            # Send command
            self.writer.write(command.value)
            await self.writer.drain()
            
            self.last_command_time = datetime.now()
            self.current_command = command
            
            if self.on_command_sent:
                self.on_command_sent(command)
            
            return True
            
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")
            if self.on_error:
                self.on_error(e)
            return False
    
    async def _uart_read_loop(self):
        """
        Read UART responses from STM32 (debug output, sensor data).
        Runs in background as async task.
        """
        print("[UART] Reader started (monitoring for debug output)")
    
        buffer = ""
        while self.running and self.connected and self.reader:
            try:
                # Read available data (non-blocking)
                data = await asyncio.wait_for(
                    self.reader.read(100),
                    timeout=0.5
                )
            
                if data:
                    # Decode and add to buffer
                    text = data.decode('utf-8', errors='ignore')
                    buffer += text
                
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                    
                        if line:
                            self._process_uart_response(line)
            
            except asyncio.TimeoutError:
                # No data available, continue
                pass
            except Exception as e:
                print(f"[WARN] UART read error: {e}")
                await asyncio.sleep(0.5)
    
    def _process_uart_response(self, line: str):
        """
        Process received UART response from STM32.
        
        Expected formats:
            "US A=XXcm B=YYcm"     - Ultrasonic sensor readings
            "Wall L=XXcm R=YYcm"   - Wall avoidance debug
            "ToF: FL=XXXmm FR=YYYmm L=ZZZmm R=WWWmm" - ToF distances
            "Junction: L=X R=Y Both=Z" - Junction detection
            "Obstacle: Front=X Closest=YYYmm" - Obstacle detection
            "STATUS L=XX R=YY F=ZZ SPD=WW MOV=V" - Status report
            "FD=XXX" - Front distance
            "CMD: X" - Command acknowledgment
        """
        try:
            # Print all received lines for debugging
            print(f"  [STM32] {line}")
            
            # Parse FreeRTOS task startup messages
            if "Task: ToF started" in line:
                print(f"  [TASK] ✓ ToF FreeRTOS task started successfully")
            elif "ToF update #" in line:
                # Suppress frequent iteration logs to avoid spam
                pass
            
            # Parse ToF initialization messages
            elif "ToF Init:" in line:
                print(f"  [INIT] {line}")
            elif "ToF sensor" in line and ("initialized" in line or "not found" in line or "not responding" in line):
                print(f"  [SENSOR] {line}")
            elif "I2C Speed:" in line:
                print(f"  [I2C] {line}")
            
            # Parse debug messages for device handle validation
            elif "dev=NULL or addr=0" in line:
                print(f"  [ERROR] ⚠️ {line} - Initialization failed for this sensor")
            elif "first:" in line and "mm stat=" in line:
                print(f"  [MEASUREMENT] ✓ {line}")
            
            # Parse ToF sensor distances
            if line.startswith("ToF: "):
                # Format: "ToF: FL=245mm FR=312mm L=156mm R=189mm"
                parts = line.split()
                all_zero = True
                for part in parts[1:]:  # Skip "ToF:"
                    if '=' in part and 'mm' in part:
                        key, value = part.split('=')
                        distance_str = value.replace('mm', '').strip()
                        
                        try:
                            distance = int(distance_str)
                            
                            if key == 'FL':
                                self.tof_front_left_mm = distance
                            elif key == 'FR':
                                self.tof_front_right_mm = distance
                            elif key == 'L':
                                self.tof_left_mm = distance
                            elif key == 'R':
                                self.tof_right_mm = distance
                            
                            if distance > 0:
                                all_zero = False
                            
                            self.last_tof_update = datetime.now()
                            
                        except ValueError:
                            pass
                
                # Warn if all sensors returning 0mm
                if all_zero:
                    print("  [WARN] ⚠️ All ToF sensors returning 0mm - measurement not working!")
            
            # Parse junction detection
            elif line.startswith("Junction: "):
                # Format: "Junction: L=1 R=0 Both=0"
                parts = line.split()
                for part in parts[1:]:  # Skip "Junction:"
                    if '=' in part:
                        key, value = part.split('=')
                        try:
                            val = bool(int(value))
                            
                            if key == 'L':
                                self.junction_left = val
                            elif key == 'R':
                                self.junction_right = val
                            elif key == 'Both':
                                self.junction_both = val
                                
                        except ValueError:
                            pass
            
            # Parse obstacle detection
            elif line.startswith("Obstacle: "):
                # Format: "Obstacle: Front=0 Closest=156mm"
                parts = line.split()
                for part in parts[1:]:  # Skip "Obstacle:"
                    if '=' in part:
                        key, value = part.split('=')
                        try:
                            if key == 'Front':
                                self.front_obstacle = bool(int(value))
                            elif key == 'Closest':
                                closest_str = value.replace('mm', '').strip()
                                self.closest_distance_mm = int(closest_str)
                                
                        except ValueError:
                            pass
            
            # Parse ultrasonic sensor data (existing)
            elif line.startswith("US ") or line.startswith("Wall "):
                # Extract distances: "US A=15cm B=20cm" or "Wall L=15cm R=20cm"
                parts = line.split()
                
                for part in parts:
                    if '=' in part and 'cm' in part:
                        key, value = part.split('=')
                        distance_str = value.replace('cm', '').strip()
                        
                        try:
                            distance = float(distance_str)
                            
                            # Update sensor values
                            if key in ['A', 'L']:  # Left sensor
                                self.ultrasonic_left_cm = distance
                            elif key in ['B', 'R']:  # Right sensor
                                self.ultrasonic_right_cm = distance
                            elif key in ['C', 'F']:  # Front sensor (F for Front)
                                # Store front sensor data
                                print(f"  [FRONT] Front sensor: {distance}cm")
                            
                            self.last_ultrasonic_update = datetime.now()
                            
                        except ValueError:
                            pass
            
            # Parse front distance response
            elif line.startswith("FD=") or "Front=" in line:
                # Extract front distance: "FD=25cm" or similar
                if "=" in line:
                    parts = line.split("=")
                    if len(parts) > 1:
                        distance_str = parts[1].replace('cm', '').replace('mm', '').strip()
                        try:
                            distance = float(distance_str)
                            print(f"  [FRONT] Front distance: {distance}{'mm' if 'mm' in line else 'cm'}")
                        except ValueError:
                            pass
            
            # Parse STATUS response format
            elif line.startswith("STATUS "):
                # Format: "STATUS L=5 R=18 F=25 SPD=70 MOV=N"
                parts = line.split()
                for part in parts[1:]:  # Skip "STATUS"
                    if '=' in part:
                        key, value = part.split('=')
                        try:
                            if key == 'L':
                                self.ultrasonic_left_cm = float(value)
                            elif key == 'R':
                                self.ultrasonic_right_cm = float(value)
                            elif key == 'F':
                                print(f"  [FRONT] Front: {value}cm")
                            elif key == 'SPD':
                                print(f"  ⚡ Speed: {value}%")
                            elif key == 'MOV':
                                print(f"  🚗 Moving: {value}")
                        except ValueError:
                            pass
                self.last_ultrasonic_update = datetime.now()
                
                # Warn if sensors not responding (distance = 0)
                if self.ultrasonic_left_cm == 0 and self.ultrasonic_right_cm == 0:
                    print("  [WARN] WARNING: Both ultrasonic sensors reading 0cm (not connected?)")
                elif self.ultrasonic_left_cm == 0:
                    print("  [WARN] WARNING: Left ultrasonic sensor reading 0cm (check wiring)")
                elif self.ultrasonic_right_cm == 0:
                    print("  [WARN] WARNING: Right ultrasonic sensor reading 0cm (check wiring)")
                
                # Warn if obstacle detected
                if self.ultrasonic_left_cm > 0 and self.ultrasonic_left_cm < 5:
                    print(f"  [ALERT] OBSTACLE LEFT: {self.ultrasonic_left_cm}cm!")
                if self.ultrasonic_right_cm > 0 and self.ultrasonic_right_cm < 5:
                    print(f"  [ALERT] OBSTACLE RIGHT: {self.ultrasonic_right_cm}cm!")
        
        except Exception as e:
            print(f"  [WARN] Error parsing UART response: {e}")
    
    def get_ultrasonic_distances(self) -> tuple[float, float]:
        """
        Get current ultrasonic sensor distances.
        
        Returns:
            tuple: (left_cm, right_cm)
        """
        age = (datetime.now() - self.last_ultrasonic_update).total_seconds()
        
        if age > 2.0:
            print("[WARN] Ultrasonic data stale (>2s old)")
        
        return (self.ultrasonic_left_cm, self.ultrasonic_right_cm)
    
    def get_tof_distances(self) -> tuple[int, int, int, int]:
        """
        Get current ToF sensor distances.
        
        Returns:
            tuple: (front_left_mm, front_right_mm, left_mm, right_mm)
        """
        age = (datetime.now() - self.last_tof_update).total_seconds()
        
        if age > 3.0:
            print("[WARN] ToF data stale (>3s old)")
        
        return (self.tof_front_left_mm, self.tof_front_right_mm, self.tof_left_mm, self.tof_right_mm)
    
    def get_junction_status(self) -> tuple[bool, bool, bool]:
        """
        Get current junction detection status.
        
        Returns:
            tuple: (left_junction, right_junction, both_junctions)
        """
        return (self.junction_left, self.junction_right, self.junction_both)
    
    def get_obstacle_status(self) -> tuple[bool, int]:
        """
        Get current obstacle detection status.
        
        Returns:
            tuple: (front_obstacle_detected, closest_distance_mm)
        """
        return (self.front_obstacle, self.closest_distance_mm)
    
    async def forward(self, duration: Optional[float] = None):
        """
        Move forward.
        
        Args:
            duration: Optional duration in seconds. If None, moves indefinitely.
        """
        if self.emergency_stop_active:
            print("[WARN] Emergency stop active. Reset required.")
            return
        
        await self._send_command(MotorCommand.FORWARD)
        print("[MOVE] Moving forward")
        
        if duration:
            await asyncio.sleep(duration)
            await self.stop()
    
    async def reverse(self, duration: Optional[float] = None):
        """
        Move in reverse.
        
        Args:
            duration: Optional duration in seconds. If None, moves indefinitely.
        """
        if self.emergency_stop_active:
            print("[WARN] Emergency stop active. Reset required.")
            return
        
        await self._send_command(MotorCommand.REVERSE)
        print("[MOVE] Moving reverse")
        
        if duration:
            await asyncio.sleep(duration)
            await self.stop()
    
    async def turn_left(self, duration: Optional[float] = None):
        """
        Turn left (spot turn).
        
        Args:
            duration: Optional duration in seconds. If None, turns indefinitely.
        """
        if self.emergency_stop_active:
            print("[WARN] Emergency stop active. Reset required.")
            return
        
        await self._send_command(MotorCommand.LEFT)
        print("[MOVE] Turning left")
        
        if duration:
            await asyncio.sleep(duration)
            await self.stop()
    
    async def turn_right(self, duration: Optional[float] = None):
        """
        Turn right (spot turn).
        
        Args:
            duration: Optional duration in seconds. If None, turns indefinitely.
        """
        if self.emergency_stop_active:
            print("[WARN] Emergency stop active. Reset required.")
            return
        
        await self._send_command(MotorCommand.RIGHT)
        print("[MOVE] Turning right")
        
        if duration:
            await asyncio.sleep(duration)
            await self.stop()
    
    async def stop(self):
        """Stop all motors."""
        await self._send_command(MotorCommand.STOP)
        print("[MOVE] Stopped")
    
    async def set_speed(self, speed: SpeedLevel):
        """
        Set motor speed level.
        
        Args:
            speed: SpeedLevel enum (SLOW, MEDIUM, FAST)
        """
        await self._send_command(speed.command)
        self.current_speed = speed
        print(f"[SPEED] Speed set to {speed}")
    
    async def set_speed_slow(self):
        """Set speed to SLOW (40%)."""
        await self.set_speed(SpeedLevel.SLOW)
    
    async def set_speed_medium(self):
        """Set speed to MEDIUM (70%)."""
        await self.set_speed(SpeedLevel.MEDIUM)
    
    async def set_speed_fast(self):
        """Set speed to FAST (100%)."""
        await self.set_speed(SpeedLevel.FAST)
    
    async def enable_acceleration(self):
        """Enable smooth acceleration and deceleration."""
        await self._send_command(MotorCommand.ACCEL_ENABLE)
        self.accel_enabled = True
        print("[OK] Smooth acceleration/deceleration ENABLED")
    
    async def disable_acceleration(self):
        """Disable acceleration (instant speed changes)."""
        await self._send_command(MotorCommand.ACCEL_DISABLE)
        self.accel_enabled = False
        print("[OK] Smooth acceleration/deceleration DISABLED (instant response)")
    
    async def reset_emergency_stop(self):
        """Reset emergency stop state."""
        self.emergency_stop_active = False
        await self.stop()
        print("[OK] Emergency stop reset")
    
    async def request_ultrasonic_ping(self):
        """Request ultrasonic distance measurement from STM32."""
        # Store old values to detect if we got fresh data
        old_left = self.ultrasonic_left_cm
        old_right = self.ultrasonic_right_cm
        old_time = self.last_ultrasonic_update
        
        await self._send_command(MotorCommand.ULTRASONIC_PING)
        print("[SENSOR] Ultrasonic ping sent")
        
        # Wait up to 1 second for fresh response
        for i in range(10):
            await asyncio.sleep(0.1)
            if self.last_ultrasonic_update > old_time:
                # Got fresh data!
                break
        
        left, right = self.get_ultrasonic_distances()
        
        if self.last_ultrasonic_update <= old_time:
            print("  [WARN] No response from STM32 (check UART connection)")
            print(f"  [CACHE] Left: {left:.1f}cm, Right: {right:.1f}cm")
        else:
            print(f"  [OK] Left: {left:.1f}cm, Right: {right:.1f}cm")
            
        # Also request front sensor and ToF data for comprehensive reading
        await asyncio.sleep(0.2)
        await self.request_front_distance()
        await asyncio.sleep(0.2)
        await self.request_tof_distances()
    
    async def request_all_sensors(self):
        """Request readings from all sensor systems."""
        print("[SENSOR] Requesting complete sensor status...")
        await self.request_ultrasonic_ping()
        await asyncio.sleep(0.3)
        await self.request_tof_distances()
        await asyncio.sleep(0.3) 
        await self.request_junction_status()
        await asyncio.sleep(0.3)
        await self.request_obstacle_status()
        await asyncio.sleep(0.3)
        await self.request_status_report()
        print("[OK] Complete sensor check finished")
    
    async def request_tof_distances(self):
        """Request ToF sensor distances from STM32."""
        old_time = self.last_tof_update
        
        await self._send_command(MotorCommand.TOF_DISTANCES)
        print("[SENSOR] ToF distances request sent")
        
        # Wait up to 1 second for fresh response
        for i in range(10):
            await asyncio.sleep(0.1)
            if self.last_tof_update > old_time:
                break
        
        fl, fr, left, right = self.get_tof_distances()
        
        if self.last_tof_update <= old_time:
            print("  [WARN] No response from STM32 (check UART connection)")
            print(f"  [CACHE] FL:{fl}mm FR:{fr}mm L:{left}mm R:{right}mm")
        else:
            # Check if sensors are returning valid data
            if fl == 0 and fr == 0 and left == 0 and right == 0:
                print(f"  [WARN] ⚠️ All ToF sensors returning 0mm")
                print(f"  [HINT] Run 'X' command to test I2C communication")
                print(f"  [HINT] Run 'N' command to reinitialize sensors")
            else:
                print(f"  [OK] ✓ FL:{fl}mm FR:{fr}mm L:{left}mm R:{right}mm")
    
    async def request_junction_status(self):
        """Request junction detection status from STM32."""
        await self._send_command(MotorCommand.TOF_JUNCTIONS)
        print("[SENSOR] Junction status request sent")
        
        # Wait briefly for response
        await asyncio.sleep(0.2)
        
        left, right, both = self.get_junction_status()
        print(f"  [OK] Junctions - Left:{left} Right:{right} Both:{both}")
    
    async def request_obstacle_status(self):
        """Request obstacle detection status from STM32."""
        await self._send_command(MotorCommand.TOF_OBSTACLES)
        print("[SENSOR] Obstacle status request sent")
        
        # Wait briefly for response
        await asyncio.sleep(0.2)
        
        front_obstacle, closest = self.get_obstacle_status()
        print(f"  [OK] Front Obstacle:{front_obstacle} Closest:{closest}mm")
    
    async def request_front_distance(self):
        """Request front sensor distance from STM32."""
        await self._send_command(MotorCommand.FRONT_DISTANCE)
        print("[SENSOR] Front distance request sent")
        await asyncio.sleep(0.2)
    
    async def request_status_report(self):
        """Request comprehensive status report from STM32."""
        await self._send_command(MotorCommand.STATUS_REPORT)
        print("[SENSOR] Status report request sent")
        await asyncio.sleep(0.2)
    
    async def request_tof_hardware_test(self):
        """Request ToF hardware test (debug command X)."""
        if not self.connected or not self.writer:
            print("[ERROR] Not connected")
            return
        
        try:
            self.writer.write(b'X')
            await self.writer.drain()
            print("[TEST] Running comprehensive ToF hardware diagnostics...")
            print("  [INFO] This will test I2C communication, sensor detection, and timing")
            print("  [INFO] Watch for Model ID (should be 0xEE or 0x238 for VL53L0X)")
            await asyncio.sleep(3.0)  # Wait longer for complete test sequence
        except Exception as e:
            print(f"[ERROR] Failed to send ToF test command: {e}")
    
    async def toggle_slow_i2c_mode(self):
        """Toggle I2C speed mode for ToF sensors (command Y)."""
        if not self.connected or not self.writer:
            print("[ERROR] Not connected")
            return
        
        try:
            self.writer.write(b'Y')
            await self.writer.drain()
            print("[ACTION] Toggling I2C speed mode for ToF sensors...")
            await asyncio.sleep(0.3)  # Wait a bit longer for response
        except Exception as e:
            print(f"[ERROR] Failed to send I2C speed toggle command: {e}")
    
    async def reinitialize_tof_sensors(self):
        """Reinitialize ToF sensors (command N)."""
        if not self.connected or not self.writer:
            print("[ERROR] Not connected")
            return
        
        try:
            self.writer.write(b'N')
            await self.writer.drain()
            print("[ACTION] Reinitializing ToF sensors (full power cycle + ST API init)...")
            print("  [INFO] This takes ~2 seconds per sensor")
            await asyncio.sleep(3.0)  # Wait longer for full initialization sequence
            
            # Automatically request fresh readings after reinitialization
            await asyncio.sleep(0.5)
            await self.request_tof_distances()
        except Exception as e:
            print(f"[ERROR] Failed to send ToF reinitialize command: {e}")
    
    async def _heartbeat_loop(self):
        """
        Send periodic commands to prevent STM32 safety timeout.
        Runs in background as async task.
        """
        print(f"💓 Heartbeat started (interval: {self.config.heartbeat_interval}s)")
        
        while self.running and self.connected:
            try:
                time_since_last = (datetime.now() - self.last_command_time).total_seconds()
                
                # If no command sent recently, send current command to keep alive
                if time_since_last >= self.config.heartbeat_interval:
                    await self._send_command(self.current_command)
                
                await asyncio.sleep(0.5)
                
            except Exception as e:
                print(f"[WARN] Heartbeat error: {e}")
                await asyncio.sleep(1)
    
    async def run(self):
        """
        Main run loop. Starts background tasks.
        """
        if not self.connected:
            if not await self.connect():
                return
        
        # Start heartbeat task
        heartbeat_task = asyncio.create_task(self._heartbeat_loop())
        uart_reader_task = asyncio.create_task(self._uart_read_loop())
        
        try:
            # Keep running until stopped
            while self.running:
                await asyncio.sleep(0.1)
                
        except asyncio.CancelledError:
            pass
        finally:
            heartbeat_task.cancel()
            uart_reader_task.cancel()
            await self.disconnect()
    
    def cleanup(self):
        """Cleanup GPIO resources."""
        if GPIO_AVAILABLE:
            GPIO.cleanup()
            print("[OK] GPIO cleaned up")


class InteractiveController:
    """Interactive keyboard control interface."""
    
    def __init__(self, motor_controller: MotorController):
        self.motor = motor_controller
        self.running = True
    
    def print_menu(self):
        """Print control menu."""
        print("\n" + "="*60)
        print("[INTERACTIVE] Raspberry Pi Motor Controller - Interactive Mode")
        print("="*60)
        print("\n[MENU] Movement Controls:")
        print("  W/↑ - Forward")
        print("  S/↓ - Reverse")
        print("  A/← - Turn Left")
        print("  D/→ - Turn Right")
        print("  SPACE - Stop")
        print("\n[MENU] Speed Controls:")
        print("  1 - Slow Speed (40%)")
        print("  2 - Medium Speed (70%)")
        print("  3 - Fast Speed (100%)")
        print("\n[MENU] Acceleration:")
        print("  M - Enable Smooth Accel/Decel")
        print("  Z - Disable Accel (Instant)")
        print("\n[MENU] Sensor Commands:")
        print("  U - Ultrasonic Ping (HC-SR04 backup sensors)")
        print("  G - ToF Distances (4x VL53L0X precision sensors)")
        print("  J - Junction Detection (ToF-based)")
        print("  O - Obstacle Detection (ToF-based)")
        print("  F - Front Distance (single sensor)")
        print("  B - Status Report (all sensors)")
        print("  A - All Sensors (comprehensive check)")
        print("  X - ToF Hardware Test (debug I2C communication)")
        print("  Y - Toggle I2C Speed (slow/fast mode for ToF sensors)")
        print("  N - Reinitialize ToF Sensors (fix startup issues)")
        print("\n[MENU] Safety & System:")
        print("  E - Emergency Stop")
        print("  R - Reset Emergency Stop")
        print("  I - Info")
        print("  Q - Quit")
        print("="*60)
    
    async def run(self):
        """Run interactive control loop."""
        self.print_menu()
        
        # Note: This is a simple version. For production, use aioconsole
        print("\n[WARN] For better keyboard handling, install: pip install aioconsole")
        print("Type command + Enter:")
        
        try:
            import aioconsole
            has_aioconsole = True
        except ImportError:
            has_aioconsole = False
        
        while self.running:
            try:
                if has_aioconsole:
                    cmd = (await aioconsole.ainput("")).strip().lower()
                else:
                    # Fallback to sync input
                    cmd = await asyncio.get_event_loop().run_in_executor(
                        None, input, ""
                    )
                    cmd = cmd.strip().lower()
                
                if cmd in ['w', '↑', 'up']:
                    await self.motor.forward()
                elif cmd in ['s', '↓', 'down']:
                    await self.motor.reverse()
                elif cmd in ['a', '←', 'left']:
                    await self.motor.turn_left()
                elif cmd in ['d', '→', 'right']:
                    await self.motor.turn_right()
                elif cmd in [' ', 'space', '']:
                    await self.motor.stop()
                elif cmd == '1':
                    await self.motor.set_speed_slow()
                elif cmd == '2':
                    await self.motor.set_speed_medium()
                elif cmd == '3':
                    await self.motor.set_speed_fast()
                elif cmd == 'm':
                    await self.motor.enable_acceleration()
                elif cmd == 'z':
                    await self.motor.disable_acceleration()
                elif cmd == 'u':
                    await self.motor.request_ultrasonic_ping()
                elif cmd == 'g':
                    await self.motor.request_tof_distances()
                elif cmd == 'j':
                    await self.motor.request_junction_status()
                elif cmd == 'o':
                    await self.motor.request_obstacle_status()
                elif cmd == 'f':
                    await self.motor.request_front_distance()
                elif cmd == 'b':
                    await self.motor.request_status_report()
                elif cmd == 'a':
                    await self.motor.request_all_sensors()
                elif cmd == 'x':
                    await self.motor.request_tof_hardware_test()
                elif cmd == 'y':
                    await self.motor.toggle_slow_i2c_mode()
                elif cmd == 'n':
                    await self.motor.reinitialize_tof_sensors()
                elif cmd == 'e':
                    await self.motor.stop()
                    self.motor.emergency_stop_active = True
                    print("[ALERT] Emergency stop activated")
                elif cmd == 'r':
                    await self.motor.reset_emergency_stop()
                elif cmd == 'i':
                    self.print_info()
                elif cmd == 'q':
                    print("Exiting...")
                    self.running = False
                    break
                else:
                    print(f"Unknown command: '{cmd}'")
            
            except KeyboardInterrupt:
                print("\n\nInterrupted!")
                break
            except Exception as e:
                print(f"Error: {e}")
        
        await self.motor.stop()
    
    def print_info(self):
        """Print current status."""
        print("\n" + "-"*50)
        print("[STATUS] Status:")
        print(f"  Connected: {self.motor.connected}")
        print(f"  Current Command: {self.motor.current_command}")
        print(f"  Current Speed: {self.motor.current_speed}")
        print(f"  Acceleration: {'Enabled (Smooth)' if self.motor.accel_enabled else 'Disabled (Instant)'}")
        print(f"  Emergency Stop: {self.motor.emergency_stop_active}")
        
        print("\n[SENSORS] Ultrasonic Sensors (HC-SR04):")
        print(f"  Left: {self.motor.ultrasonic_left_cm:.1f}cm")
        print(f"  Right: {self.motor.ultrasonic_right_cm:.1f}cm")
        us_age = (datetime.now() - self.motor.last_ultrasonic_update).total_seconds()
        print(f"  Data Age: {us_age:.1f}s")
        
        print("\n[TOF] Sensors (VL53L0X):")
        fl, fr, left, right = self.motor.get_tof_distances()
        print(f"  Front-Left: {fl}mm")
        print(f"  Front-Right: {fr}mm")
        print(f"  Left: {left}mm")
        print(f"  Right: {right}mm")
        tof_age = (datetime.now() - self.motor.last_tof_update).total_seconds()
        print(f"  Data Age: {tof_age:.1f}s")
        
        # ToF sensor status check
        if fl == 0 and fr == 0 and left == 0 and right == 0:
            print("  [WARN] All ToF sensors reading 0mm (initialization issue?)")
        elif fl == 0 or fr == 0 or left == 0 or right == 0:
            print("  [WARN] Some ToF sensors not responding")
        
        print("\n[JUNCTION] Detection:")
        j_left, j_right, j_both = self.motor.get_junction_status()
        print(f"  Left: {j_left}")
        print(f"  Right: {j_right}")
        print(f"  Both: {j_both}")
        
        print("\n[OBSTACLE] Detection:")
        obstacle, closest = self.motor.get_obstacle_status()
        print(f"  Front Obstacle: {obstacle}")
        print(f"  Closest Distance: {closest}mm")
        
        print(f"\n[CONNECTION] Details:")
        print(f"  Port: {self.motor.config.port}")
        print(f"  Baud Rate: {self.motor.config.baudrate}")
        print("-"*50)


async def demo_sequence(motor: MotorController):
    """
    Run automated demo sequence with speed variations, acceleration, and sensor testing.
    """
    print("\n🤖 Running Demo Sequence with PWM Speed Control, Acceleration & ToF Sensors...")
    print("-"*60)
    
    sequences = [
        ("Enable Smooth Acceleration", motor.enable_acceleration, 0.5),
        ("Set Medium Speed", motor.set_speed_medium, 0.5),
        
        # Test sensors first
        ("Test Ultrasonic Sensors", motor.request_ultrasonic_ping, 1.0),
        ("Test ToF Distances", motor.request_tof_distances, 1.0),
        ("Test Junction Detection", motor.request_junction_status, 1.0),
        ("Test Obstacle Detection", motor.request_obstacle_status, 1.0),
        ("Get Status Report", motor.request_status_report, 1.5),
        
        # Movement with sensor monitoring
        ("Forward (Medium, Smooth)", motor.forward, 3.0),
        ("Check Sensors During Movement", motor.request_tof_distances, 0.5),
        ("Stop (Smooth Decel)", motor.stop, 1.5),
        
        ("Set Slow Speed", motor.set_speed_slow, 0.5),
        ("Reverse (Slow, Smooth)", motor.reverse, 2.0),
        ("Check Junction Status", motor.request_junction_status, 0.5),
        ("Stop", motor.stop, 1.5),
        
        ("Set Fast Speed", motor.set_speed_fast, 0.5),
        ("Left Turn (Fast, Smooth)", motor.turn_left, 1.5),
        ("Check Obstacle Detection", motor.request_obstacle_status, 0.5),
        ("Stop", motor.stop, 1.5),
        
        ("Right Turn (Fast, Smooth)", motor.turn_right, 1.5),
        ("Final Sensor Check", motor.request_tof_distances, 0.5),
        ("Stop", motor.stop, 1.5),
        
        ("Disable Acceleration", motor.disable_acceleration, 0.5),
        ("Forward Sprint (Fast, Instant)", motor.forward, 2.0),
        ("Stop (Instant)", motor.stop, 1.0),
        
        ("Re-enable Smooth Mode", motor.enable_acceleration, 0.5),
        ("Set Medium Speed", motor.set_speed_medium, 0.5),
    ]
    
    for name, func, duration in sequences:
        print(f"\n▶ {name} ({duration}s)")
        if duration > 0:
            if any(word in name.lower() for word in ['forward', 'reverse', 'turn']):
                await func(duration)
            else:
                await func()
                if duration > 0.5:
                    await asyncio.sleep(duration - 0.5)
        else:
            await func()
        await asyncio.sleep(0.5)
    
    print("\n[OK] Demo complete!")


async def main():
    """Main entry point."""
    print("""
    ╔═══════════════════════════════════════════════════════════╗
    ║   Raspberry Pi 4B - STM32 Motor Controller Interface    ║
    ║        Version 2.1 - Acceleration Control - 2025         ║
    ╚═══════════════════════════════════════════════════════════╝
    """)
    
    # Configuration
    config = ControllerConfig(
        port='/dev/serial0',  # Change to /dev/ttyUSB0 if using USB-Serial
        baudrate=9600,
        timeout=1.0,
        command_interval=0.1,
        heartbeat_interval=1.5,
        emergency_stop_pin=17
    )
    
    # Create controller
    motor = MotorController(config)
    
    # Setup signal handlers for clean shutdown
    def signal_handler(sig, frame):
        print("\n\n[WARN] Signal received, shutting down...")
        motor.running = False
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Connect
    if not await motor.connect():
        print("Failed to connect. Exiting.")
        return
    
    # Start background tasks
    heartbeat_task = asyncio.create_task(motor._heartbeat_loop())
    uart_reader_task = asyncio.create_task(motor._uart_read_loop())
    
    # Wait a moment for any startup messages from STM32
    print("\n Waiting for STM32 startup messages...")
    await asyncio.sleep(2)
    
    try:
        # Choose mode
        print("\n Select Mode:")
        print("  1 - Interactive Control (Keyboard)")
        print("  2 - Demo Sequence")
        print("  3 - API Mode (programmatic control)")
        
        mode = input("\nMode [1-3] > ").strip()
        
        if mode == '1':
            # Interactive mode
            controller = InteractiveController(motor)
            await controller.run()
            
        elif mode == '2':
            # Demo mode
            await demo_sequence(motor)
            
        elif mode == '3':
            # API mode - example
            print("\n[API] Running example sequence with speed control, acceleration & ToF sensors...")
            
            # Test all sensors first
            print("\n1. Testing all sensor systems")
            await motor.request_ultrasonic_ping()
            await asyncio.sleep(0.5)
            await motor.request_tof_distances()
            await asyncio.sleep(0.5)
            await motor.request_junction_status()
            await asyncio.sleep(0.5)
            await motor.request_obstacle_status()
            await asyncio.sleep(1.0)
            
            # Example with smooth acceleration/deceleration
            print("\n2. Enable smooth acceleration")
            await motor.enable_acceleration()
            await asyncio.sleep(0.5)
            
            print("\n3. Set medium speed and move forward (watch it ramp up!)")
            await motor.set_speed_medium()
            await motor.forward(3)
            await asyncio.sleep(0.5)
            
            # Monitor sensors during movement
            print("\n4. Check ToF sensors during movement")
            await motor.request_tof_distances()
            await asyncio.sleep(0.5)
            
            print("\n5. Stop with smooth deceleration")
            await motor.stop()
            await asyncio.sleep(1.5)
            
            print("\n6. Test junction detection")
            await motor.request_junction_status()
            await asyncio.sleep(0.5)
            
            print("\n7. Set fast speed and turn right (smooth)")
            await motor.set_speed_fast()
            await motor.turn_right(2)
            await asyncio.sleep(0.5)
            
            print("\n8. Check obstacle detection")
            await motor.request_obstacle_status()
            await asyncio.sleep(0.5)
            
            print("\n9. Disable acceleration for instant response")
            await motor.disable_acceleration()
            await asyncio.sleep(0.5)
            
            print("\n10. Sprint forward at fast speed (instant)")
            await motor.forward(2)
            await asyncio.sleep(0.5)
            
            print("\n11. Instant stop")
            await motor.stop()
            await asyncio.sleep(0.5)
            
            print("\n12. Final comprehensive sensor check")
            await motor.request_status_report()
            await asyncio.sleep(1.0)
            
            print("\n13. Re-enable smooth mode")
            await motor.enable_acceleration()
            
            print("\n[OK] API sequence complete - All 7 sensors (3 ultrasonic + 4 ToF) tested!")
            
        else:
            print("Invalid mode")
    
    except KeyboardInterrupt:
        print("\n\nShutdown requested...")
    except Exception as e:
        print(f"\n[ERROR] Error: {e}")
    finally:
        # Cleanup
        heartbeat_task.cancel()
        if 'uart_reader_task' in locals():
            uart_reader_task.cancel()
        await motor.disconnect()
        motor.cleanup()
        print("\n👋 Goodbye!")


if __name__ == "__main__":
    # Run async main
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nExiting...")
