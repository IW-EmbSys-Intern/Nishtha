import time
from pymavlink import mavutil

print("Connecting to vehicle...")
master = mavutil.mavlink_connection('udpin:localhost:14550')

print("Waiting for heartbeat...")
master.wait_heartbeat()

print(f"Connected: sys={master.target_system}, comp={master.target_component}")

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

    print(f"Switching to {mode}...")

    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if mavutil.mode_string_v10(msg) == mode:
            print(f"Mode {mode} confirmed")
            return

def arm():
    print("Arming...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )


def takeoff(altitude):
    print(f"Taking off to {altitude}m...")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude
    )

def wait_for_position(timeout=60):
    print("Waiting for GPS/EKF fix...")

    start = time.time()

    while time.time() - start < timeout:
        msg = master.recv_match(
            type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
            blocking=True,
            timeout=1
        )

        if not msg:
            continue

        if msg.get_type() == 'GPS_RAW_INT':
            if msg.fix_type >= 3:
                print("GPS fix OK")

        if msg.get_type() == 'GLOBAL_POSITION_INT':
            print("Position OK")
            return True

    return False

def send_velocity(vx, vy, vz=0):
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )

print("Allowing EKF to initialize...")
time.sleep(5)

if not wait_for_position():
    raise Exception("No position estimate")

set_mode("GUIDED")
time.sleep(1)

arm()
time.sleep(2)

takeoff_alt = 10
takeoff(takeoff_alt)

print("Waiting for takeoff...")

while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    alt = msg.relative_alt / 1000.0
    print(f"Altitude: {alt:.2f} m")

    if alt >= takeoff_alt * 0.95:
        print("Takeoff complete")
        break

    time.sleep(0.5)


print("Moving forward...")

forward_speed = 2  # m/s
duration = 30       # seconds

start_time = time.time()

while time.time() - start_time < duration:
    send_velocity(forward_speed, 0, 0)
    time.sleep(0.1)


print("Stopping...")
send_velocity(0, 0, 0)

print("Mission complete")
