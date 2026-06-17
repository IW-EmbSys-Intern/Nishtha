import time
from pymavlink import mavutil

# Connect to SITL
print("Connecting to vehicle...")
master = mavutil.mavlink_connection('udpin:localhost:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()
print(
    f"Connected to system {master.target_system}, "
    f"component {master.target_component}"
)

def set_mode(mode):
    mode_mapping = master.mode_mapping()

    if mode not in mode_mapping:
        raise Exception(f"Unknown mode: {mode}")

    mode_id = mode_mapping[mode]

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print(f"Requested mode change to {mode}")

    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        current_mode = mavutil.mode_string_v10(msg)

        if current_mode == mode:
            print(f"Mode changed to {mode}")
            return

def wait_for_position_estimate(timeout=60):
    print("Waiting for GPS/EKF position estimate...")

    start = time.time()

    while time.time() - start < timeout:

        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
            blocking=True,
            timeout=1
        )

        if msg is None:
            continue

        if msg.get_type() == 'GPS_RAW_INT':
            if msg.fix_type >= 3:
                print(f"GPS Fix Type: {msg.fix_type}")

        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print("Position estimate available")
            return True

    return False

def arm_vehicle():
    print("Arming vehicle...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)

        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Vehicle armed")
            return

def takeoff(altitude):
    print(f"Taking off to {altitude} meters")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude
    )
print("Allowing EKF to initialize...")
time.sleep(10)

if not wait_for_position_estimate():
    raise Exception(
        "No position estimate available. "
        "Check GPS/EKF status in SITL."
    )

set_mode("GUIDED")

time.sleep(2)

arm_vehicle()

time.sleep(2)

target_altitude = 10

takeoff(target_altitude)

while True:
    msg = master.recv_match(
        type='GLOBAL_POSITION_INT',
        blocking=True
    )

    altitude = msg.relative_alt / 1000.0

    print(f"Altitude: {altitude:.2f} m")

    if altitude >= target_altitude * 0.95:
        print("Target altitude reached")
        break

    time.sleep(1)

print("Mission complete")
