from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

vehicle = connect("127.0.0.1:14550", wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        if alt >= aTargetAltitude * 0.95:
            break
        time.sleep(1)

def get_location_offset_m(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    new_lat = original_location.lat + (dLat * 180 / math.pi)
    new_lon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)

def goto(loc):
    vehicle.simple_goto(loc)
    time.sleep(8)

arm_and_takeoff(35)

start = vehicle.location.global_relative_frame
p100 = get_location_offset_m(start, 100, 0)
goto(p100)

base = vehicle.location.global_relative_frame

radius_dist = 30
angle45 = math.radians(45)

center = get_location_offset_m(base, radius_dist * math.cos(angle45), radius_dist * math.sin(angle45))

steps = 39

for i in range(steps):
    angle = 2 * math.pi * i / steps
    dNorth = center.alt * 0 + radius_dist * math.cos(angle)
    dEast = radius_dist * math.sin(angle)
    point = get_location_offset_m(center, dNorth, dEast)
    point.alt = base.alt
    vehicle.simple_goto(point)
    time.sleep(1.2)

vehicle.mode = VehicleMode("RTL")

while vehicle.armed:
    time.sleep(3)

vehicle.close()
