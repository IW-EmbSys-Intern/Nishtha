import asyncio
import sys
import tty
import termios
from mavsdk import System
from mavsdk.action import ActionError

drone = System()

async def init_drone():
    """Connects to PX4 SITL and blocks until GPS checks pass."""
    print("Connecting to PX4 SITL...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for drone to get a strong 3D GPS Lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("GPS Lock OK! Pre-arm checks passed.")
            break

async def execute_action(coro, action_name):
    """Executes a MAVSDK action and catches internal PX4 errors."""
    try:
        await coro
        print(f"[Success] {action_name} executed successfully.")
    except ActionError as e:
        print(f"[PX4 Error] Failed to {action_name}: {e}")

async def handle_key(key):
    """Processes keystrokes smoothly inside the main event loop."""
    char = key.lower()
    
    if char == 'a':
        print("\n[Action] Requesting Arming...")
        await execute_action(drone.action.arm(), "Arm")
    elif char == 't':
        print("\n[Action] Requesting Takeoff...")
        await execute_action(drone.action.takeoff(), "Takeoff")
    elif char == 'l':
        print("\n[Action] Requesting Landing...")
        await execute_action(drone.action.land(), "Land")
    elif key == '\x1b':
        print("\nExiting keyboard control script.")
        return False
    return True

async def main():
    await init_drone()
    
    print("\n=========================")
    print("=== KEYBOARD CONTROLS ===")
    print("Press 'A' to Arm")
    print("Press 'T' to Takeoff")
    print("Press 'L' to Land")
    print("Press 'ESC' to Quit")
    print("=========================\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    
    try:
        tty.setraw(sys.stdin.fileno())
        loop = asyncio.get_running_loop()
        reader = asyncio.StreamReader()
        protocol = asyncio.StreamReaderProtocol(reader)
        await loop.connect_read_pipe(lambda: protocol, sys.stdin)

        while True:
            res = await reader.read(1)
            key = res.decode()
            
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
            continue_running = await handle_key(key)
            
            tty.setraw(sys.stdin.fileno())
            
            if not continue_running:
                break
                
    finally:
  
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nScript interrupted by user.")
