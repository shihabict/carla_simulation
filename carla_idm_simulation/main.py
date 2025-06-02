import carla
import random
import time
import csv
import math

# ==== CARLA Client Setup ====
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town01')
bp_lib = world.get_blueprint_library()

# ==== Weather Setup ====
weather = carla.WeatherParameters.ClearNoon
world.set_weather(weather)

# ==== Spawn Points ====
spawn_points = world.get_map().get_spawn_points()

# ==== Select Spawn Locations ====
leader_spawn = spawn_points[80]
follower_spawn = spawn_points[78]  # same lane, ~10-15m behind

# ==== Vehicle Blueprints ====
leader_bp = bp_lib.find('vehicle.tesla.model3')
follower_bp = bp_lib.find('vehicle.audi.etron')

# ==== Spawn Vehicles ====
leader = world.try_spawn_actor(leader_bp, leader_spawn)
follower = world.try_spawn_actor(follower_bp, follower_spawn)

if not leader or not follower:
    raise Exception("Failed to spawn vehicles. Try different spawn points.")

print("Leader and Follower spawned.")

# ==== Set Autopilot for Leader ====
leader.set_autopilot(True)

# ==== Spectator View Behind Follower ====
spectator = world.get_spectator()
cam_offset = carla.Location(x=-6, z=2.5)
spectator.set_transform(carla.Transform(follower.get_transform().transform(cam_offset), follower.get_transform().rotation))

# ==== CSV Logging Setup ====
log_file = open("car_following_log.csv", mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow(["time", "leader_x", "leader_y", "follower_x", "follower_y", "leader_speed", "follower_speed", "gap", "rel_vel", "accel"])
