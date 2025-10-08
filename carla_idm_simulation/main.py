import carla
import random
import time
import csv
import math
import os
import pygame
from data_logger import DataLogger
from camera_setup import SmoothCamera
from idm_controller import IDMController
from follower_controller import FollowerController

# ==== CARLA Client Setup ====
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town04')
bp_lib = world.get_blueprint_library()
tm = client.get_trafficmanager()

# ==== Weather Setup ====
world.set_weather(carla.WeatherParameters.CloudyNoon)

# ==== Spawn Points ====
spawn_points = world.get_map().get_spawn_points()

leader_index = 8
follower_index = leader_index + 2  # usually right behind in same lane
leader_spawn = spawn_points[leader_index]
leader_location = leader_spawn.location
leader_rotation = leader_spawn.rotation

# ==== Vehicle Blueprints ====
leader_bp = bp_lib.find('vehicle.tesla.model3')
follower_bp = bp_lib.find('vehicle.audi.tt')

# Compute a location 10 meters behind the leader
offset = carla.Location(x=-10)  # behind by 10 meters in local coordinates
follower_location = leader_spawn.transform(offset)
follower_transform = carla.Transform(follower_location, leader_spawn.rotation)

# ==== Spawn Vehicles ====
leader = world.try_spawn_actor(leader_bp, leader_spawn)
follower = world.try_spawn_actor(follower_bp, follower_transform)

# ===== ADD PHYSICS DAMPING HERE TO REDUCE JITTERNESS =====
physics_control = follower.get_physics_control()
# 1. Increase damping forces
physics_control.linear_damping = 0.3    # Reduces forward/backward oscillations (Default: 0.0)
physics_control.angular_damping = 0.7   # Reduces rotational shaking (Default: 0.0)

# 2. Stabilize suspension
for wheel in physics_control.wheels:
    wheel.damping_rate = 2.0             # Default: 0.25 (higher = less bounce)
    wheel.max_steer_angle = 70.0        # Default: 70 (reduce if twitchy)
follower.apply_physics_control(physics_control)


# ==== Set Autopilot for Leader ====
leader.set_autopilot(True, tm.get_port())
# follower.set_autoplot(False)

# Setup spectator view behind follower
spectator = world.get_spectator()

follower_tf = follower.get_transform()
follower_location = follower_tf.location
follower_rotation = follower_tf.rotation

# Get forward vector and move back 8 meters, up 3 meters
back_vector = follower_tf.get_forward_vector() * -16  # move behind
up_vector = carla.Location(z=3)  # camera height

spectator_location = follower_location + back_vector + up_vector
spectator_transform = carla.Transform(spectator_location, follower_rotation)

spectator.set_transform(spectator_transform)

# from idm_controller import IDMController

# Initialize IDM controller (custom logic you define)
idm = IDMController()
controller = FollowerController(world, follower, leader, idm)
logger = DataLogger()

# while True:
#     controller.update()
#     follower_tf = follower.get_transform()
#
#     spectator = world.get_spectator()
#     back_vecactor = follower_tf.get_forward_vector() * -8
#     camera_location = follower_tf.location + back_vector + carla.Location(z=3)
#     camera_tf = carla.Transform(camera_location, follower_tf.rotation)
#     spectator.set_transform(camera_tf)

try:
    while True:
        controller.update()
        follower_tf = follower.get_transform()

        # Calculate logging parameters
        dx = leader.get_transform().location.distance(follower.get_transform().location)
        leader_vel = leader.get_velocity()
        follower_vel = follower.get_velocity()
        dv = math.sqrt(leader_vel.x**2 + leader_vel.y**2) - math.sqrt(follower_vel.x**2 + follower_vel.y**2)

        # Log data
        logger.log_data(leader, follower, dx, dv)

        # Camera update (existing code)
        spectator = world.get_spectator()
        back_vector = follower_tf.get_forward_vector() * -8
        camera_location = follower_tf.location + back_vector + carla.Location(z=3)
        camera_tf = carla.Transform(camera_location, follower_tf.rotation)
        spectator.set_transform(camera_tf)

except KeyboardInterrupt:
    print("Simulation stopped by user")
finally:
    logger.stop_logging()
    print("Data saved to logs/car_following_data.csv")