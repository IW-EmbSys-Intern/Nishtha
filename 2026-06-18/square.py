from dronekit import connect
from dronekit import VehicleMode
from dronekit import LocationGlobalRelative
import time
import math

vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(target_altitude):
	while not vehicle.is_armable:
		print("waiting for vehicle...")
		time.sleep(1)
	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode.name != "GUIDED":
		time.sleep(1)
	vehicle.armed = True
	while not vehicle.armed:
		time.sleep(1)
	print("Taking Off")
	
	vehicle.simple_takeoff(target_altitude)

	while True:
		current_alt = vehicle.location.global_relative_frame.alt

		print("Altitude:", current_alt)
		if current_alt > target_altitude * 0.95:
			print("Target alttitude reached")
			break
		time.sleep(1)

def get_location_meters(original_location, dNorth, dEast):
    earth_radius = 6378137.0

    dLat = dNorth / earth_radius
    dLon = dEast / (
        earth_radius * math.cos(math.pi * original_location.lat / 180)
    )

    new_lat = original_location.lat + (dLat * 180 / math.pi)
    new_lon = original_location.lon + (dLon * 180 / math.pi)

    return LocationGlobalRelative(
        new_lat,
        new_lon,
        original_location.alt
    )
def goto_position(dNorth, dEast):
	current = vehicle.location.global_relative_frame

	target = get_location_meters(current, dNorth, dEast)
	vehicle.simple_goto(target)
	time.sleep(10)
arm_and_takeoff(35)

goto_position(40, 0)

#square
goto_position(0, 30)
goto_position(-30, 0)
goto_position(0, -30)
goto_position(30, 0)

#land
vehicle.mode = VehicleMode("LAND")
vehicle.close()
