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
        '1' - Set speed to SLOW (50%)
        '2' - Set speed to MEDIUM (70%)
        '3' - Set speed to FAST (100%)
    
    Acceleration Control:
        'M' - Enable smooth acceleration/deceleration
        'Z' - Disable acceleration (instant speed changes)
        'Y' - Disable acceleration (alternate)

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
    ACCEL_DISABLE_ALT = b'Y'  # Alternate disable command (matches firmware change)
    ULTRASONIC_PING = b'U'  # Request ultrasonic distance measurement
    TOF_PING = b'o'         # Request ToF distances (left/right 45° sensors)
    
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
    SLOW = (50, MotorCommand.SPEED_SLOW)      # 50% PWM (raised from 40%)
    MEDIUM = (70, MotorCommand.SPEED_MEDIUM)  # 70% PWM
    FAST = (100, MotorCommand.SPEED_FAST)     # 100% PWM
    
    def __init__(self, percentage: int, command: MotorCommand):
        self.percentage = percentage
        self.command = command
    
    def __str__(self):
        return f"{self.name} ({self.percentage}%)"

# Turn timing constants (match firmware scaling: TURN_90_MS * 100 / speed%)
# Keep this in sync with firmware `Core/Inc/main.h` (default 700 ms)
TURN_90_MS = 350          # base ms for 90-degree turn at 100% speed (match firmware; user tuned)
TURN_90_MS_MIN = 150
TURN_90_MS_MAX = 5000


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
        self.ultrasonic_front_cm = 0.0
        self.last_ultrasonic_update = datetime.now()
        # ToF sensor data (mm)
        self.tof_left_mm = 0
        self.tof_right_mm = 0
        self.last_tof_update = datetime.now()
        
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
            print(f"✓ Emergency stop button configured on GPIO {self.config.emergency_stop_pin}")
        except Exception as e:
            print(f"⚠️  GPIO setup warning: {e}")
            print(f"   Emergency stop button on GPIO {self.config.emergency_stop_pin} may not be available")
            # Don't fail - continue without GPIO emergency stop
    
    def _emergency_stop_callback(self, channel):
        """Handle emergency stop button press."""
        print("\n🚨 EMERGENCY STOP ACTIVATED!")
        self.emergency_stop_active = True
        asyncio.create_task(self.stop())
    
    async def connect(self) -> bool:
        """
        Connect to STM32 motor controller.
        
        Returns:
            bool: True if connection successful
        """
        try:
            print(f"🔌 Connecting to {self.config.port} at {self.config.baudrate} baud...")
            
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
            
            print(f"✓ Connected to motor controller")
            
            if self.on_connect:
                self.on_connect()
            
            return True
            
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            if self.on_error:
                self.on_error(e)
            return False
    
    async def disconnect(self):
        """Disconnect from motor controller."""
        if self.connected:
            print("\n🔌 Disconnecting...")
            
            # Send stop command before disconnecting
            await self.stop()
            await asyncio.sleep(0.1)
            
            if self.writer:
                self.writer.close()
                await self.writer.wait_closed()
            
            self.connected = False
            self.running = False
            
            print("✓ Disconnected")
            
            if self.on_disconnect:
                self.on_disconnect()
    
    async def _send_command(self, command: MotorCommand, update_current: bool = True) -> bool:
        """
        Send command to motor controller.
        
        Args:
            command: Motor command to send
            
        Returns:
            bool: True if command sent successfully
        """
        if not self.connected or not self.writer:
            print("✗ Not connected")
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
            # Only update the persistent current command when requested.
            if update_current:
                self.current_command = command
            
            if self.on_command_sent:
                self.on_command_sent(command)
            
            return True
            
        except Exception as e:
            print(f"✗ Failed to send command: {e}")
            if self.on_error:
                self.on_error(e)
            return False
    
    async def _uart_read_loop(self):
        """
        Read UART responses from STM32 (debug output, sensor data).
        Runs in background as async task.
        """
        print("📥 UART reader started (monitoring for debug output)")
    
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
                print(f"⚠️  UART read error: {e}")
                await asyncio.sleep(0.5)
    
    def _process_uart_response(self, line: str):
        """
        Process received UART response from STM32.
        
        Expected formats:
            "US A=XXcm B=YYcm"  - Ultrasonic sensor readings
            "Wall L=XXcm R=YYcm" - Wall avoidance debug
            "CMD: X" - Command acknowledgment
        """
        try:
            # Process ultrasonic sensor data differently depending on message type.
            # Only print distances when a full ping (A/B/C) is received or when explicitly requested.
            # Otherwise, silently update left/right to avoid spamming the console.

            # Full-format ping from STM32 uses 'A=.. B=.. C=..' (we print these)
            if line.startswith("US A=") or (' C=' in line) or line.startswith("US ") and 'C=' in line:
                # Print the full ping line and parse distances
                print(f"  📩 STM32: {line}")
                parts = line.split()
                for part in parts:
                    if '=' in part and 'cm' in part:
                        key, value = part.split('=')
                        distance_str = value.replace('cm', '').strip()
                        try:
                            distance = float(distance_str)
                            if key in ['A', 'L']:
                                self.ultrasonic_left_cm = distance
                            elif key in ['B', 'R']:
                                self.ultrasonic_right_cm = distance
                            elif key in ['C', 'F']:
                                self.ultrasonic_front_cm = distance
                            self.last_ultrasonic_update = datetime.now()
                        except ValueError:
                            pass

                # Print concise summary for the full ping
                left, right, front = self.get_ultrasonic_distances()
                print(f"  ✓ Left: {left:.1f}cm, Right: {right:.1f}cm, Front: {front:.1f}cm")

            # Periodic short-format status 'US L=.. R=..' — update silently (no repeated prints)
            elif line.startswith("US "):
                parts = line.split()
                for part in parts:
                    if '=' in part and 'cm' in part:
                        key, value = part.split('=')
                        distance_str = value.replace('cm', '').strip()
                        try:
                            distance = float(distance_str)
                            if key in ['A', 'L']:
                                self.ultrasonic_left_cm = distance
                            elif key in ['B', 'R']:
                                self.ultrasonic_right_cm = distance
                            # do NOT overwrite front unless explicitly provided
                            self.last_ultrasonic_update = datetime.now()
                        except ValueError:
                            pass

            # Wall messages may include L/R or other debug info — print for visibility
            elif line.startswith("Wall "):
                print(f"  📩 STM32: {line}")
                parts = line.split()
                for part in parts:
                    if '=' in part and 'cm' in part:
                        key, value = part.split('=')
                        distance_str = value.replace('cm', '').strip()
                        try:
                            distance = float(distance_str)
                            if key in ['L', 'A']:
                                self.ultrasonic_left_cm = distance
                            elif key in ['R', 'B']:
                                self.ultrasonic_right_cm = distance
                            self.last_ultrasonic_update = datetime.now()
                        except ValueError:
                            pass

            else:
                # For other messages (ACKs, CMD:, ToF replies, debug) process/print appropriately.
                # Detect ToF reply format (case-insensitive) and parse robustly
                lower = line.lower()
                if lower.startswith("tof ") or lower.startswith("tof l=") or "tof " in lower:
                    print(f"  📩 STM32: {line}")
                    parts = line.replace(',', ' ').split()
                    parsed = False
                    for part in parts:
                        if '=' in part:
                            key, value = part.split('=', 1)
                            key = key.strip().upper()
                            # Strip non-digits from value (allow optional 'mm')
                            valstr = ''.join(ch for ch in value if ch.isdigit())
                            if not valstr:
                                continue
                            try:
                                v = int(valstr)
                            except ValueError:
                                continue
                            parsed = True
                            if key.startswith('L'):
                                self.tof_left_mm = v
                            elif key.startswith('R'):
                                self.tof_right_mm = v
                            self.last_tof_update = datetime.now()

                    if parsed:
                        # Interpret 8191 as OUT_OF_RANGE (firmware sentinel)
                        left = self.tof_left_mm
                        right = self.tof_right_mm
                        if left == 8191 or right == 8191:
                            lstr = 'OUT_OF_RANGE' if left == 8191 else f'{left}mm'
                            rstr = 'OUT_OF_RANGE' if right == 8191 else f'{right}mm'
                            print(f"  ✓ ToF Left: {lstr}, Right: {rstr}")
                        else:
                            # Warn if values are zero (likely no reply)
                            if left == 0 and right == 0:
                                print("  ⚠️  ToF values are 0 — possible missing reply or parsing issue")
                            print(f"  ✓ ToF Left: {left}mm, Right: {right}mm")
                    else:
                        # Couldn't parse numeric values — print raw line for debugging
                        print(f"  ⚠️  Unparsed ToF line: {line}")
                else:
                    # Junction notification from firmware: "JUNC L=... R=..."
                    if 'junc' in lower:
                        print(f"  ⚠️  Junction detected by STM32: {line}")
                        # Auto-acknowledge the junction to the STM32 so it can continue
                        # with the handshake. Send single-byte 'K' acknowledgement.
                        try:
                            asyncio.create_task(self._send_raw(b'K'))
                            print("  → Sent ACK to STM32 for junction")
                        except Exception as e:
                            print(f"  ⚠️  Failed to send ACK: {e}")
                        print("  ▶ Wait for operator to send direction ('a'/'d') or use API to send.")
                    else:
                    # Generic debug/ACK lines
                    print(f"  📩 STM32: {line}")

        except Exception as e:
            print(f"  ⚠️  Error parsing UART response: {e}")

    async def _send_raw(self, data: bytes) -> bool:
        """
        Send raw bytes over UART without modifying the persistent current command.
        """
        if not self.connected or not self.writer:
            print("✗ Not connected")
            return False
        try:
            time_since_last = (datetime.now() - self.last_command_time).total_seconds()
            if time_since_last < self.config.command_interval:
                await asyncio.sleep(self.config.command_interval - time_since_last)
            self.writer.write(data)
            await self.writer.drain()
            self.last_command_time = datetime.now()
            return True
        except Exception as e:
            print(f"✗ Failed to send raw data: {e}")
            if self.on_error:
                self.on_error(e)
            return False
    
    def get_ultrasonic_distances(self) -> tuple[float, float]:
        """
        Get current ultrasonic sensor distances.
        
        Returns:
            tuple: (left_cm, right_cm, front_cm)
        """
        age = (datetime.now() - self.last_ultrasonic_update).total_seconds()
        
        if age > 2.0:
            print("⚠️  Ultrasonic data stale (>2s old)")
        
        return (self.ultrasonic_left_cm, self.ultrasonic_right_cm, self.ultrasonic_front_cm)
    
    async def forward(self, duration: Optional[float] = None):
        """
        Move forward.
        
        Args:
            duration: Optional duration in seconds. If None, moves indefinitely.
        """
        if self.emergency_stop_active:
            print("⚠️  Emergency stop active. Reset required.")
            return
        
        # If this is a timed move, don't make it persistent so heartbeat
        # doesn't re-send the movement command after the duration expires.
        if duration:
            await self._send_command(MotorCommand.FORWARD, update_current=False)
        else:
            await self._send_command(MotorCommand.FORWARD)
        print("▲ Moving forward")
        
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
            print("⚠️  Emergency stop active. Reset required.")
            return
        
        if duration:
            await self._send_command(MotorCommand.REVERSE, update_current=False)
        else:
            await self._send_command(MotorCommand.REVERSE)
        print("▼ Moving reverse")
        
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
            print("⚠️  Emergency stop active. Reset required.")
            return
        
        if duration:
            # Timed turn should not become the persistent command
            await self._send_command(MotorCommand.LEFT, update_current=False)
        else:
            await self._send_command(MotorCommand.LEFT)
        print("◄ Turning left")
        
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
            print("⚠️  Emergency stop active. Reset required.")
            return
        
        if duration:
            await self._send_command(MotorCommand.RIGHT, update_current=False)
        else:
            await self._send_command(MotorCommand.RIGHT)
        print("► Turning right")
        
        if duration:
            await asyncio.sleep(duration)
            await self.stop()
    
    async def stop(self):
        """Stop all motors."""
        await self._send_command(MotorCommand.STOP)
        print("■ Stopped")
    
    async def set_speed(self, speed: SpeedLevel):
        """
        Set motor speed level.
        
        Args:
            speed: SpeedLevel enum (SLOW, MEDIUM, FAST)
        """
        await self._send_command(speed.command)
        self.current_speed = speed
        print(f"🏃 Speed set to {speed}")
    
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
        print("✓ Smooth acceleration/deceleration ENABLED")
    
    async def disable_acceleration(self):
        """Disable acceleration (instant speed changes)."""
        await self._send_command(MotorCommand.ACCEL_DISABLE)
        self.accel_enabled = False
        print("✓ Smooth acceleration/deceleration DISABLED (instant response)")
    
    async def reset_emergency_stop(self):
        """Reset emergency stop state."""
        self.emergency_stop_active = False
        await self.stop()
        print("✓ Emergency stop reset")
    
    async def request_ultrasonic_ping(self):
        """Request ultrasonic distance measurement from STM32."""
        # Store old values to detect if we got fresh data
        old_left = self.ultrasonic_left_cm
        old_right = self.ultrasonic_right_cm
        old_front = self.ultrasonic_front_cm
        old_time = self.last_ultrasonic_update
        
        await self._send_command(MotorCommand.ULTRASONIC_PING)
        print("📡 Ultrasonic ping sent")
        
        # Wait up to 1 second for fresh response
        for i in range(10):
            await asyncio.sleep(0.1)
            if self.last_ultrasonic_update > old_time:
                # Got fresh data!
                break
        
        left, right, front = self.get_ultrasonic_distances()

        if self.last_ultrasonic_update <= old_time:
            print("  ⚠️  No response from STM32 (check UART connection)")
            print(f"  💾 Cached values: Left: {left:.1f}cm, Right: {right:.1f}cm, Front: {front:.1f}cm")
        else:
            print(f"  ✓ Left: {left:.1f}cm, Right: {right:.1f}cm, Front: {front:.1f}cm")

    async def request_tof_ping(self):
        """Request ToF distances (left/right 45°) from STM32."""
        old_time = self.last_tof_update

        attempts = 3
        for attempt in range(1, attempts + 1):
            await self._send_command(MotorCommand.TOF_PING)
            print(f"📡 ToF ping sent (attempt {attempt}/{attempts})")

            # Wait up to 2 seconds for fresh response
            waited = 0.0
            timeout = 2.0
            while waited < timeout:
                await asyncio.sleep(0.1)
                waited += 0.1
                if self.last_tof_update > old_time:
                    break

            if self.last_tof_update > old_time:
                # Got fresh data
                # Interpret 8191 as OUT_OF_RANGE (firmware sentinel)
                left = self.tof_left_mm
                right = self.tof_right_mm
                if left == 8191 or right == 8191:
                    lstr = 'OUT_OF_RANGE' if left == 8191 else f'{left}mm'
                    rstr = 'OUT_OF_RANGE' if right == 8191 else f'{right}mm'
                    print(f"  ✓ ToF Left: {lstr}, Right: {rstr}")
                else:
                    print(f"  ✓ ToF Left: {left}mm, Right: {right}mm")
                return

            # No fresh data this attempt
            print(f"  ⚠️  No ToF response (attempt {attempt}). Retrying...")

        # After retries, still no response
        print("  ✗ ToF: no response from STM32 after multiple attempts")
        print(f"  💾 Last cached ToF: Left: {self.tof_left_mm}mm, Right: {self.tof_right_mm}mm")

    def compute_turn_duration(self) -> float:
        """
        Compute a turn duration (seconds) for an approximate 90-degree spot turn
        using the same scaling the firmware uses: turn_ms = (TURN_90_MS * 100) / speed%

        Returns:
            float: duration in seconds (clamped to min/max)
        """
        try:
            speed_pct = int(self.current_speed.percentage)
        except Exception:
            speed_pct = 70

        if speed_pct <= 0:
            speed_pct = 70

        turn_ms = (TURN_90_MS * 100) / speed_pct
        # clamp
        if turn_ms < TURN_90_MS_MIN:
            turn_ms = TURN_90_MS_MIN
        if turn_ms > TURN_90_MS_MAX:
            turn_ms = TURN_90_MS_MAX

        return float(turn_ms) / 1000.0
    
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
                print(f"⚠️  Heartbeat error: {e}")
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
            print("✓ GPIO cleaned up")


class InteractiveController:
    """Interactive keyboard control interface."""
    
    def __init__(self, motor_controller: MotorController):
        self.motor = motor_controller
        self.running = True
    
    def print_menu(self):
        """Print control menu."""
        print("\n" + "="*60)
        print("🎮 Raspberry Pi Motor Controller - Interactive Mode")
        print("="*60)
        print("\n📍 Movement Controls:")
        print("  W/↑ - Forward")
        print("  S/↓ - Reverse")
        print("  A/← - Turn Left")
        print("  D/→ - Turn Right")
        print("  SPACE - Stop")
        print("\n⚡ Speed Controls:")
        print("  1 - Slow Speed (50%)")
        print("  2 - Medium Speed (70%)")
        print("  3 - Fast Speed (100%)")
        print("\n🚀 Acceleration:")
        print("  M - Enable Smooth Accel/Decel")
        print("  Z - Disable Accel (Instant)")
        print("\n📡 Sensors:")
        print("  U - Ultrasonic Ping (check wall distances)")
        print("  O - ToF Ping (left/right 45° sensors)")
        print("\n🛡 Safety & System:")
        print("  E - Emergency Stop")
        print("  R - Reset Emergency Stop")
        print("  I - Info")
        print("  Q - Quit")
        print("="*60)
    
    async def run(self):
        """Run interactive control loop."""
        self.print_menu()
        
        # Note: This is a simple version. For production, use aioconsole
        print("\n⚠️  For better keyboard handling, install: pip install aioconsole")
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
                    # Compute duration to do an approximate 90° spot turn
                    dur = self.motor.compute_turn_duration()
                    await self.motor.turn_left(dur)
                elif cmd in ['d', '→', 'right']:
                    dur = self.motor.compute_turn_duration()
                    await self.motor.turn_right(dur)
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
                elif cmd == 'o':
                    await self.motor.request_tof_ping()
                elif cmd == 'e':
                    await self.motor.stop()
                    self.motor.emergency_stop_active = True
                    print("🚨 Emergency stop activated")
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
        print("\n" + "-"*40)
        print("📊 Status:")
        print(f"  Connected: {self.motor.connected}")
        print(f"  Current Command: {self.motor.current_command}")
        print(f"  Current Speed: {self.motor.current_speed}")
        print(f"  Acceleration: {'Enabled (Smooth)' if self.motor.accel_enabled else 'Disabled (Instant)'}")
        print(f"  Emergency Stop: {self.motor.emergency_stop_active}")
        print(f"  Ultrasonic Left: {self.motor.ultrasonic_left_cm:.1f}cm")
        print(f"  Ultrasonic Right: {self.motor.ultrasonic_right_cm:.1f}cm")
        age = (datetime.now() - self.motor.last_ultrasonic_update).total_seconds()
        print(f"  Sensor Data Age: {age:.1f}s")
        print(f"  Port: {self.motor.config.port}")
        print(f"  Baud Rate: {self.motor.config.baudrate}")
        print("-"*40)


async def demo_sequence(motor: MotorController):
    """
    Run automated demo sequence with speed variations and acceleration.
    """
    print("\n🤖 Running Demo Sequence with PWM Speed Control & Acceleration...")
    print("-"*60)
    
    sequences = [
        ("Enable Smooth Acceleration", motor.enable_acceleration, 0.5),
        ("Set Medium Speed", motor.set_speed_medium, 0.5),
        ("Forward (Medium, Smooth)", motor.forward, 3.0),
        ("Stop (Smooth Decel)", motor.stop, 1.5),
        
        ("Set Slow Speed", motor.set_speed_slow, 0.5),
        ("Reverse (Slow, Smooth)", motor.reverse, 2.0),
        ("Stop", motor.stop, 1.5),
        
        ("Set Fast Speed", motor.set_speed_fast, 0.5),
        ("Left Turn (Fast, Smooth)", motor.turn_left, 1.5),
        ("Stop", motor.stop, 1.5),
        
        ("Right Turn (Fast, Smooth)", motor.turn_right, 1.5),
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
            await func(duration) if 'forward' in name.lower() or 'reverse' in name.lower() or 'turn' in name.lower() else await func()
        else:
            await func()
        await asyncio.sleep(0.5)
    
    print("\n✓ Demo complete!")


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
        print("\n\n⚠️  Signal received, shutting down...")
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
    print("\n⏳ Waiting for STM32 startup messages...")
    await asyncio.sleep(2)
    
    try:
        # Choose mode
        print("\n📋 Select Mode:")
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
            print("\n🔧 API Mode - Running example sequence with speed control & acceleration...")
            
            # Example with smooth acceleration/deceleration
            print("\n1. Enable smooth acceleration")
            await motor.enable_acceleration()
            await asyncio.sleep(0.5)
            
            print("\n2. Set medium speed and move forward (watch it ramp up!)")
            await motor.set_speed_medium()
            await motor.forward(3)
            await asyncio.sleep(0.5)
            
            print("\n3. Stop with smooth deceleration")
            await motor.stop()
            await asyncio.sleep(1.5)
            
            print("\n4. Set fast speed and turn right (smooth)")
            await motor.set_speed_fast()
            await motor.turn_right(2)
            await asyncio.sleep(0.5)
            
            print("\n5. Disable acceleration for instant response")
            await motor.disable_acceleration()
            await asyncio.sleep(0.5)
            
            print("\n6. Sprint forward at fast speed (instant)")
            await motor.forward(2)
            await asyncio.sleep(0.5)
            
            print("\n7. Instant stop")
            await motor.stop()
            await asyncio.sleep(0.5)
            
            print("\n8. Re-enable smooth mode")
            await motor.enable_acceleration()
            
            print("\n✓ API sequence complete")
            
        else:
            print("Invalid mode")
    
    except KeyboardInterrupt:
        print("\n\nShutdown requested...")
    except Exception as e:
        print(f"\n✗ Error: {e}")
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
