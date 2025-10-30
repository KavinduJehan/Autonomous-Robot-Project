#!/usr/bin/env python3
"""
Simple Robot Control Examples
==============================
Easy-to-use examples for controlling the robot from Raspberry Pi.

Usage:
    python3 simple_examples.py
"""

import asyncio
import sys
from rpi_motor_controller import MotorController, ControllerConfig


async def example_1_basic_movement():
    """
    Example 1: Basic Movement
    Simple forward, turn, reverse sequence.
    """
    print("\n" + "="*60)
    print("Example 1: Basic Movement")
    print("="*60)
    
    motor = MotorController(ControllerConfig(port='/dev/serial0'))
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        print("Moving forward for 3 seconds...")
        await motor.forward(3)
        
        print("Turning right for 1 second...")
        await motor.turn_right(1)
        
        print("Moving forward for 2 seconds...")
        await motor.forward(2)
        
        print("Stopping...")
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 1 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


async def example_2_square_pattern():
    """
    Example 2: Square Pattern
    Drive in a square shape.
    """
    print("\n" + "="*60)
    print("Example 2: Square Pattern")
    print("="*60)
    
    motor = MotorController()
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        for i in range(4):
            print(f"Side {i+1}/4...")
            await motor.forward(2)
            await asyncio.sleep(0.3)
            await motor.turn_right(0.9)  # 90-degree turn
            await asyncio.sleep(0.3)
        
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 2 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


async def example_3_zigzag():
    """
    Example 3: Zigzag Pattern
    Move in zigzag pattern.
    """
    print("\n" + "="*60)
    print("Example 3: Zigzag Pattern")
    print("="*60)
    
    motor = MotorController()
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        for i in range(3):
            print(f"Zigzag {i+1}/3...")
            await motor.forward(1.5)
            await motor.turn_right(0.5)
            await motor.forward(1.5)
            await motor.turn_left(0.5)
        
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 3 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


async def example_4_distance_based():
    """
    Example 4: Simulated Distance-Based Movement
    (In real application, use ultrasonic sensor)
    """
    print("\n" + "="*60)
    print("Example 4: Distance-Based Control")
    print("="*60)
    
    motor = MotorController()
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        # Simulate obstacle detection after random time
        import random
        
        print("Moving forward until 'obstacle'...")
        start_time = asyncio.get_event_loop().time()
        obstacle_time = start_time + random.uniform(2, 4)
        
        await motor.forward()
        
        while asyncio.get_event_loop().time() < obstacle_time:
            await asyncio.sleep(0.1)
        
        print("Obstacle detected! Backing up...")
        await motor.stop()
        await asyncio.sleep(0.3)
        
        await motor.reverse(1)
        
        print("Turning to avoid...")
        await motor.turn_right(1)
        
        print("Continuing...")
        await motor.forward(2)
        
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 4 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


async def example_5_timed_patrol():
    """
    Example 5: Timed Patrol Route
    Follow a patrol pattern with specific timings.
    """
    print("\n" + "="*60)
    print("Example 5: Timed Patrol Route")
    print("="*60)
    
    motor = MotorController()
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        patrol_route = [
            ("Forward", motor.forward, 3),
            ("Right Turn", motor.turn_right, 1),
            ("Forward", motor.forward, 2),
            ("Left Turn", motor.turn_left, 1),
            ("Forward", motor.forward, 3),
            ("U-Turn", motor.turn_right, 2),
        ]
        
        for step, (name, action, duration) in enumerate(patrol_route, 1):
            print(f"Step {step}/{len(patrol_route)}: {name}")
            await action(duration)
            await asyncio.sleep(0.5)  # Brief pause between steps
        
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 5 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


async def example_6_responsive_control():
    """
    Example 6: Responsive Control with Input
    Control robot based on simulated sensor input.
    """
    print("\n" + "="*60)
    print("Example 6: Responsive Control")
    print("="*60)
    print("Simulating sensor-based navigation...")
    
    motor = MotorController()
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        # Simulate 10 seconds of responsive control
        import random
        end_time = asyncio.get_event_loop().time() + 10
        
        while asyncio.get_event_loop().time() < end_time:
            # Simulate sensor reading
            sensor_value = random.choice(['clear', 'left_obstacle', 'right_obstacle'])
            
            if sensor_value == 'clear':
                await motor.forward()
                print("Path clear - moving forward")
                await asyncio.sleep(1)
                
            elif sensor_value == 'left_obstacle':
                print("Left obstacle - turning right")
                await motor.turn_right(0.5)
                await asyncio.sleep(0.5)
                
            elif sensor_value == 'right_obstacle':
                print("Right obstacle - turning left")
                await motor.turn_left(0.5)
                await asyncio.sleep(0.5)
        
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 6 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


async def example_7_gradual_movements():
    """
    Example 7: Gradual Movements
    Smooth transitions between movements.
    """
    print("\n" + "="*60)
    print("Example 7: Gradual Movements")
    print("="*60)
    
    motor = MotorController()
    
    try:
        await motor.connect()
        heartbeat = asyncio.create_task(motor._heartbeat_loop())
        
        # Forward with brief stops
        print("Gradual forward movement...")
        for i in range(5):
            await motor.forward(0.5)
            await motor.stop()
            await asyncio.sleep(0.2)
        
        await asyncio.sleep(0.5)
        
        # Gradual turn
        print("Gradual right turn...")
        for i in range(3):
            await motor.turn_right(0.3)
            await motor.stop()
            await asyncio.sleep(0.2)
        
        await motor.stop()
        
        heartbeat.cancel()
        await motor.disconnect()
        print("✓ Example 7 complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()


def print_menu():
    """Print example menu."""
    print("\n" + "="*60)
    print("🤖 Robot Control Examples")
    print("="*60)
    print("1. Basic Movement (forward, turn, stop)")
    print("2. Square Pattern")
    print("3. Zigzag Pattern")
    print("4. Distance-Based Control (simulated)")
    print("5. Timed Patrol Route")
    print("6. Responsive Control (simulated sensors)")
    print("7. Gradual Movements")
    print("0. Exit")
    print("="*60)


async def main():
    """Main menu."""
    examples = {
        '1': example_1_basic_movement,
        '2': example_2_square_pattern,
        '3': example_3_zigzag,
        '4': example_4_distance_based,
        '5': example_5_timed_patrol,
        '6': example_6_responsive_control,
        '7': example_7_gradual_movements,
    }
    
    while True:
        print_menu()
        choice = input("\nSelect example [0-7] > ").strip()
        
        if choice == '0':
            print("Goodbye!")
            break
        
        if choice in examples:
            try:
                await examples[choice]()
                input("\nPress Enter to continue...")
            except KeyboardInterrupt:
                print("\n\nExample interrupted!")
                input("Press Enter to continue...")
            except Exception as e:
                print(f"\nError: {e}")
                input("Press Enter to continue...")
        else:
            print("Invalid choice!")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)
