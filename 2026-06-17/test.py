import time
from pymavlink import mavutil

# 1. Connect to the SITL drone on port 14550 or 14551
print("Connecting to drone via PyMAVLink...")
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

# Wait for a heartbeat signal to make sure it is alive
master.wait_heartbeat()
print("Heartbeat received! Connected.")

def set_mode(mode_name):
    """Changes the flight mode."""
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

# 2. Arm and Takeoff to 10 meters
print("Switching to GUIDED mode...")
set_mode('GUIDED')

print("Arming motors...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

time.sleep(2) # Give it a brief moment to finish arming

print("Taking off to 10 meters...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 10
)

# Wait 8 seconds to let the drone climb up
time.sleep(8)

# 3. Fly forward 400 meters at 12 m/s
print("Sending velocity command: Moving forward at 12 m/s...")
# Local NED coordinates (X = North/Forward, Y = East, Z = Down)
# We send this in a loop so the drone keeps moving forward continuously
duration = 34 # 400 meters / 12 m/s = ~33.3 seconds
start_time = time.time()

while time.time() - start_time < duration:
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, # Frame relative to drone nose
        0b0000111111000111, # Tell it to use only Velocity parameters
        0, 0, 0,           # Positions (ignored)
        12, 0, 0,          # X velocity (Forward = 12m/s), Y velocity, Z velocity
        0, 0, 0,           # Acceleration (ignored)
        0, 0               # Yaw, Yaw rate (ignored)
    )
    time.sleep(1)

# 4. Land the drone
print("Destination reached. Landing...")
set_mode('LAND')
print("Finished script.")

