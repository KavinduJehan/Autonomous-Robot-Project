#!/usr/bin/env python3
"""
Navigation Interface Module
===========================
Handles communication between ESP32 Navigator and Raspberry Pi.
Receives navigation commands and translates them to motor controller actions.

Author: Robot Project Team
Date: November 2025
Version: 1.0
"""

import asyncio
import json
import logging
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Optional, Callable, Dict, Any
import serial_asyncio


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class NavCommand(Enum):
    """Navigation command types from ESP32."""
    MOVE_FORWARD = "MOVE_FORWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"
    REACHED_WAYPOINT = "REACHED_WAYPOINT"
    REACHED_DESTINATION = "REACHED_DESTINATION"
    RECALCULATING = "RECALCULATING"
    
    def __str__(self):
        return self.value


class ConnectionType(Enum):
    """Connection method for ESP32."""
    SERIAL = "serial"
    WIFI = "wifi"


@dataclass
class NavigationMessage:
    """Parsed navigation message from ESP32."""
    command: NavCommand
    angle: float = 0.0          # Degrees for turns
    distance: float = 0.0       # Meters for forward movement
    speed: int = 70             # Speed percentage (0-100)
    timestamp: int = 0          # ESP32 timestamp (millis)
    priority: str = "normal"    # Command priority
    metadata: Dict[str, Any] = None
    
    @classmethod
    def from_json(cls, json_str: str) -> Optional['NavigationMessage']:
        """
        Parse JSON message from ESP32.
        
        Expected format:
        {
            "type": "navigation",
            "command": "TURN_LEFT",
            "angle": 90,
            "distance": 0,
            "speed": 70,
            "timestamp": 1234567890,
            "priority": "normal"
        }
        """
        try:
            data = json.loads(json_str)
            
            # Validate message type
            if data.get("type") != "navigation":
                logger.warning(f"Ignoring non-navigation message: {data.get('type')}")
                return None
            
            # Parse command
            cmd_str = data.get("command", "STOP")
            try:
                command = NavCommand(cmd_str)
            except ValueError:
                logger.error(f"Unknown command: {cmd_str}")
                return None
            
            # Create message
            return cls(
                command=command,
                angle=float(data.get("angle", 0)),
                distance=float(data.get("distance", 0)),
                speed=int(data.get("speed", 70)),
                timestamp=int(data.get("timestamp", 0)),
                priority=data.get("priority", "normal"),
                metadata=data
            )
            
        except json.JSONDecodeError as e:
            logger.error(f"JSON parse error: {e}")
            return None
        except Exception as e:
            logger.error(f"Error parsing message: {e}")
            return None


@dataclass
class StatusMessage:
    """Status message to send back to ESP32."""
    state: str                      # Current robot state
    obstacle_detected: bool
    position: Dict[str, float]      # {"x": 1.5, "y": 2.3}
    heading: float                  # Degrees
    ultrasonic_left: float          # cm
    ultrasonic_right: float         # cm
    battery_voltage: float = 0.0    # Volts (if available)
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        data = {
            "type": "status",
            "state": self.state,
            "obstacle_detected": self.obstacle_detected,
            "position": self.position,
            "heading": self.heading,
            "ultrasonic_left": self.ultrasonic_left,
            "ultrasonic_right": self.ultrasonic_right,
            "battery_voltage": self.battery_voltage,
            "timestamp": int(datetime.now().timestamp() * 1000)
        }
        return json.dumps(data)


class NavigationInterface:
    """
    Interface for ESP32 navigation communication.
    Supports both Serial and WiFi connections.
    """
    
    def __init__(
        self,
        connection_type: ConnectionType = ConnectionType.SERIAL,
        serial_port: str = "/dev/ttyUSB0",
        serial_baudrate: int = 115200,
        wifi_host: str = "192.168.1.100",
        wifi_port: int = 8888
    ):
        """Initialize navigation interface."""
        self.connection_type = connection_type
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate
        self.wifi_host = wifi_host
        self.wifi_port = wifi_port
        
        # Connection state
        self.connected = False
        self.running = False
        
        # Serial connection
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        
        # WiFi connection
        self.wifi_reader: Optional[asyncio.StreamReader] = None
        self.wifi_writer: Optional[asyncio.StreamWriter] = None
        
        # Message buffers
        self.receive_buffer = ""
        self.message_queue = asyncio.Queue()
        
        # Callbacks
        self.on_navigation_command: Optional[Callable[[NavigationMessage], None]] = None
        self.on_connect: Optional[Callable] = None
        self.on_disconnect: Optional[Callable] = None
        self.on_error: Optional[Callable] = None
        
        logger.info(f"Navigation Interface initialized: {connection_type.value}")
    
    async def connect(self) -> bool:
        """
        Connect to ESP32 Navigator.
        
        Returns:
            bool: True if connection successful
        """
        try:
            if self.connection_type == ConnectionType.SERIAL:
                return await self._connect_serial()
            elif self.connection_type == ConnectionType.WIFI:
                return await self._connect_wifi()
            else:
                logger.error(f"Unknown connection type: {self.connection_type}")
                return False
                
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            if self.on_error:
                self.on_error(e)
            return False
    
    async def _connect_serial(self) -> bool:
        """Connect via Serial UART."""
        logger.info(f"Connecting to ESP32 via Serial: {self.serial_port} @ {self.serial_baudrate}")
        
        try:
            self.reader, self.writer = await serial_asyncio.open_serial_connection(
                url=self.serial_port,
                baudrate=self.serial_baudrate,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            
            self.connected = True
            self.running = True
            logger.info("✓ Connected to ESP32 via Serial")
            
            if self.on_connect:
                self.on_connect()
            
            return True
            
        except Exception as e:
            logger.error(f"Serial connection failed: {e}")
            raise
    
    async def _connect_wifi(self) -> bool:
        """Connect via WiFi TCP socket."""
        logger.info(f"Connecting to ESP32 via WiFi: {self.wifi_host}:{self.wifi_port}")
        
        try:
            self.wifi_reader, self.wifi_writer = await asyncio.open_connection(
                self.wifi_host,
                self.wifi_port
            )
            
            self.connected = True
            self.running = True
            logger.info("✓ Connected to ESP32 via WiFi")
            
            if self.on_connect:
                self.on_connect()
            
            return True
            
        except Exception as e:
            logger.error(f"WiFi connection failed: {e}")
            raise
    
    async def disconnect(self):
        """Disconnect from ESP32."""
        if not self.connected:
            return
        
        logger.info("Disconnecting from ESP32...")
        self.running = False
        
        # Close appropriate connection
        if self.connection_type == ConnectionType.SERIAL and self.writer:
            self.writer.close()
            await self.writer.wait_closed()
        elif self.connection_type == ConnectionType.WIFI and self.wifi_writer:
            self.wifi_writer.close()
            await self.wifi_writer.wait_closed()
        
        self.connected = False
        logger.info("✓ Disconnected from ESP32")
        
        if self.on_disconnect:
            self.on_disconnect()
    
    async def _receive_loop(self):
        """
        Main receive loop - reads messages from ESP32.
        Runs as background task.
        """
        logger.info("ESP32 receive loop started")
        
        # Select appropriate reader
        reader = self.reader if self.connection_type == ConnectionType.SERIAL else self.wifi_reader
        
        while self.running and self.connected and reader:
            try:
                # Read data with timeout
                data = await asyncio.wait_for(
                    reader.read(256),
                    timeout=1.0
                )
                
                if not data:
                    logger.warning("ESP32 connection closed")
                    break
                
                # Decode and add to buffer
                text = data.decode('utf-8', errors='ignore')
                self.receive_buffer += text
                
                # Process complete lines (JSON messages end with newline)
                while '\n' in self.receive_buffer:
                    line, self.receive_buffer = self.receive_buffer.split('\n', 1)
                    line = line.strip()
                    
                    if line:
                        await self._process_message(line)
                
            except asyncio.TimeoutError:
                # No data, continue
                continue
            except Exception as e:
                logger.error(f"Receive error: {e}")
                if self.on_error:
                    self.on_error(e)
                await asyncio.sleep(1)
    
    async def _process_message(self, line: str):
        """
        Process received message from ESP32.
        
        Args:
            line: JSON string message
        """
        logger.debug(f"ESP32 → RPi: {line}")
        
        # Parse message
        msg = NavigationMessage.from_json(line)
        
        if msg:
            # Add to queue
            await self.message_queue.put(msg)
            
            # Trigger callback
            if self.on_navigation_command:
                self.on_navigation_command(msg)
            
            logger.info(f"📍 Navigation: {msg.command.value} "
                       f"(angle={msg.angle}°, dist={msg.distance}m, speed={msg.speed}%)")
    
    async def send_status(self, status: StatusMessage):
        """
        Send status update to ESP32.
        
        Args:
            status: StatusMessage to send
        """
        if not self.connected:
            logger.warning("Not connected to ESP32")
            return
        
        try:
            # Select appropriate writer
            writer = self.writer if self.connection_type == ConnectionType.SERIAL else self.wifi_writer
            
            if writer:
                json_str = status.to_json()
                writer.write((json_str + '\n').encode('utf-8'))
                await writer.drain()
                
                logger.debug(f"RPi → ESP32: {json_str}")
            
        except Exception as e:
            logger.error(f"Failed to send status: {e}")
            if self.on_error:
                self.on_error(e)
    
    async def get_next_command(self, timeout: float = None) -> Optional[NavigationMessage]:
        """
        Get next navigation command from queue.
        
        Args:
            timeout: Maximum time to wait (seconds). None = wait forever.
            
        Returns:
            NavigationMessage or None if timeout
        """
        try:
            if timeout:
                return await asyncio.wait_for(self.message_queue.get(), timeout=timeout)
            else:
                return await self.message_queue.get()
        except asyncio.TimeoutError:
            return None
    
    async def run(self):
        """
        Main run loop - starts receive task.
        """
        if not self.connected:
            if not await self.connect():
                return
        
        # Start receive task
        receive_task = asyncio.create_task(self._receive_loop())
        
        try:
            # Keep running until stopped
            while self.running:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass
        finally:
            receive_task.cancel()
            await self.disconnect()


# Example usage and testing
async def test_navigation_interface():
    """Test the navigation interface."""
    print("="*60)
    print("Navigation Interface Test")
    print("="*60)
    
    # Create interface (WiFi mode for testing)
    nav = NavigationInterface(
        connection_type=ConnectionType.WIFI,
        wifi_host="192.168.1.100",  # Change to your ESP32 IP
        wifi_port=8888
    )
    
    # Set up callback
    def on_command(msg: NavigationMessage):
        print(f"\n✓ Received: {msg.command.value}")
        print(f"  Angle: {msg.angle}°")
        print(f"  Distance: {msg.distance}m")
        print(f"  Speed: {msg.speed}%")
    
    nav.on_navigation_command = on_command
    
    # Connect
    try:
        print("\nConnecting to ESP32...")
        if await nav.connect():
            print("Connected! Waiting for commands...")
            
            # Start receive loop
            receive_task = asyncio.create_task(nav._receive_loop())
            
            # Wait for some commands
            for i in range(5):
                cmd = await nav.get_next_command(timeout=10.0)
                if cmd:
                    print(f"\nCommand {i+1}: {cmd.command.value}")
                    
                    # Send status back
                    status = StatusMessage(
                        state="executing",
                        obstacle_detected=False,
                        position={"x": 1.5, "y": 2.3},
                        heading=90.0,
                        ultrasonic_left=25.5,
                        ultrasonic_right=30.2
                    )
                    await nav.send_status(status)
                    print("  ✓ Status sent to ESP32")
                else:
                    print("\nTimeout waiting for command")
                    break
            
            receive_task.cancel()
            
        else:
            print("Connection failed")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted!")
    finally:
        await nav.disconnect()
        print("\nTest complete")


if __name__ == "__main__":
    # Run test
    asyncio.run(test_navigation_interface())
