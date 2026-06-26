import asyncio
import math
import sys
import termios
import tty
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionError, MissionItem, MissionPlan

# Global state trackers
drone = System()
METERS_TO_DEG = 0.000009  # Approx 1 meter in degrees

current_active_shape = None  # Options: 'square', 'circle', 'hexagon'
is_paused = False


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
        return True
    except ActionError as e:
        print(f"[PX4 Error] Failed to {action_name}: {e}")
        return False


async def get_transition_center():
    """Calculates a point 15 meters straight ahead based on current drone heading."""
    base_lat, base_lon, heading_deg = None, None, None

    async for position in drone.telemetry.position():
        base_lat = position.latitude_deg
        base_lon = position.longitude_deg
        break

    async for heading in drone.telemetry.heading():
        heading_deg = heading.heading_deg
        break

    if None in (base_lat, base_lon, heading_deg):
        print("[Error] Failed to gather position or heading telemetry.")
        return None, None

    angle_rad = math.radians(90.0 - heading_deg)

    forward_distance = 25.0
    target_lat = base_lat + (forward_distance * math.sin(angle_rad)) * METERS_TO_DEG
    target_lon = base_lon + (forward_distance * math.cos(angle_rad)) * METERS_TO_DEG

    print(
        f"[Transition] Current Heading: {heading_deg:.1f}°. Projecting center 15m straight ahead..."
    )
    return target_lat, target_lon


async def generate_and_start_mission(mission_items, name):
    """Clears, uploads, and starts any generated polygon pattern."""
    global is_paused
    mission_plan = MissionPlan(mission_items)
    try:
        try:
            await drone.mission.pause_mission()
        except MissionError:
            pass

        print(f"[Mission] Uploading new {name} configuration...")
        await drone.mission.upload_mission(mission_plan)
        await drone.mission.start_mission()
        is_paused = False
        print(f"[Success] {name} mission sequence is now running.")
    except MissionError as e:
        print(f"[Mission Error] Failed to execute {name}: {e}")


async def start_square_mission(center_lat, center_lon):
    """Generates a 20x20m square pattern relative to a center point."""
    print("[Action] Generating Square Trajectory...")
    d_lat = 20.0 * METERS_TO_DEG
    d_lon = 20.0 * METERS_TO_DEG

    items = [
        MissionItem(
            center_lat,
            center_lon,
            20.0,
            5.0,
            True,
            0.0,
            0.0,
            MissionItem.CameraAction.NONE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            center_lat + d_lat,
            center_lon,
            20.0,
            5.0,
            True,
            0.0,
            0.0,
            MissionItem.CameraAction.NONE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            center_lat + d_lat,
            center_lon + d_lon,
            20.0,
            5.0,
            True,
            0.0,
            0.0,
            MissionItem.CameraAction.NONE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            center_lat,
            center_lon + d_lon,
            20.0,
            5.0,
            True,
            0.0,
            0.0,
            MissionItem.CameraAction.NONE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            center_lat,
            center_lon,
            20.0,
            5.0,
            True,
            0.0,
            0.0,
            MissionItem.CameraAction.NONE,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            MissionItem.VehicleAction.NONE,
        ),
    ]
    await generate_and_start_mission(items, "Square")


async def start_circle_mission(center_lat, center_lon):
    """Generates a circular track with a 10m radius around a center point."""
    print("[Action] Generating Circular Path (10m Radius)...")
    items = []
    points = 16
    radius = 10.0

    for i in range(points + 1):
        angle = (2 * math.pi / points) * i
        offset_lat = (radius * math.cos(angle)) * METERS_TO_DEG
        offset_lon = (radius * math.sin(angle)) * METERS_TO_DEG
        items.append(
            MissionItem(
                center_lat + offset_lat,
                center_lon + offset_lon,
                20.0,
                5.0,
                True,
                0.0,
                0.0,
                MissionItem.CameraAction.NONE,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                MissionItem.VehicleAction.NONE,
            )
        )
    await generate_and_start_mission(items, "Circle")


async def start_hexagon_mission(center_lat, center_lon):
    """Generates a hexagon with a 15m radius around a center point."""
    print("[Action] Generating Hexagonal Path (15m Radius)...")
    items = []
    points = 6
    radius = 15.0

    for i in range(points + 1):
        angle = (2 * math.pi / points) * i
        offset_lat = (radius * math.cos(angle)) * METERS_TO_DEG
        offset_lon = (radius * math.sin(angle)) * METERS_TO_DEG
        items.append(
            MissionItem(
                center_lat + offset_lat,
                center_lon + offset_lon,
                20.0,
                5.0,
                True,
                0.0,
                0.0,
                MissionItem.CameraAction.NONE,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                MissionItem.VehicleAction.NONE,
            )
        )
    await generate_and_start_mission(items, "Hexagon")


async def handle_key(key):
    """Processes keystrokes smoothly inside the main event loop."""
    global current_active_shape, is_paused
    char = key.lower()

    if char == "a":
        print("\n[Action] Requesting Arming...")
        await execute_action(drone.action.arm(), "Arm")

    elif char == "t":
        print("\n[Action] Requesting Takeoff to 20 meters...")
        await drone.action.set_takeoff_altitude(20.0)
        await execute_action(drone.action.takeoff(), "Takeoff")

    elif char == "m":
        if current_active_shape == "square" and is_paused:
            print("\n[Action] Resuming Square Mission...")
            await drone.mission.start_mission()
            is_paused = False
        else:
            print("\n[Trigger] Initiating Square Pattern Sequence...")
            current_active_shape = "square"
            lat, lon = await get_transition_center()
            if lat:
                await start_square_mission(lat, lon)

    elif char == "c":
        if current_active_shape == "circle" and is_paused:
            print("\n[Action] Resuming Circle Mission...")
            await drone.mission.start_mission()
            is_paused = False
        else:
            print("\n[Trigger] Initiating Circle Pattern Sequence...")
            current_active_shape = "circle"
            lat, lon = await get_transition_center()
            if lat:
                await start_circle_mission(lat, lon)

    elif char == "h":
        if current_active_shape == "hexagon" and is_paused:
            print("\n[Action] Resuming Hexagon Mission...")
            await drone.mission.start_mission()
            is_paused = False
        else:
            print("\n[Trigger] Initiating Hexagon Pattern Sequence...")
            current_active_shape = "hexagon"
            lat, lon = await get_transition_center()
            if lat:
                await start_hexagon_mission(lat, lon)

    elif char == "k":
        if current_active_shape:
            print(f"\n[Action] Pausing ongoing {current_active_shape} mission...")
            try:
                await drone.mission.pause_mission()
                is_paused = True
                print("[Success] Drone is now hovering in place.")
            except MissionError as e:
                print(f"[Mission Error] Failed to pause: {e}")
        else:
            print("\n[Warning] No active mission running to pause.")

    elif char == "l":
        print("\n[Action] Requesting Land...")
        current_active_shape = None
        is_paused = False
        await execute_action(drone.action.land(), "Land")

    elif key == "R":
        print("\n[Action] Requesting Return To Launch (RTL)...")
        current_active_shape = None
        is_paused = False
        await execute_action(drone.action.return_to_launch(), "RTL")

    elif key == "\x1b":  # ESC
        print("\nExiting keyboard control script.")
        return False
    return True


async def main():
    await init_drone()

    print("\n=========================")
    print("=== DYNAMIC POLYGON SHAPE CONTROLS ===")
    print("Press 'A' to Arm")
    print("Press 'T' to Takeoff (to 20 meters)")
    print("Press 'M' to Start/Resume Square Mission")
    print("Press 'C' to Start/Resume Circle Mission")
    print("Press 'H' to Start/Resume Hexagon Mission")
    print("Press 'K' to Pause Current Shape and Hover")
    print("Press 'L' to Land immediately")
    print("Press 'Shift + R' to perform RTL")
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
