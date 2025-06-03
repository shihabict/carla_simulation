import carla
import random
import time
import csv
import math
import os
import pygame

from tutorials.follower_controller import FollowerController
from tutorials.idm_controller import IDMController

# ==== CARLA Client Setup ====
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town02')
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

# follower = world.try_spawn_actor(follower_bp, follower_transform)


# ==== Spawn Vehicles ====
leader = world.try_spawn_actor(leader_bp, leader_spawn)
follower = world.try_spawn_actor(follower_bp, follower_transform)

# ==== Set Autopilot for Leader ====
leader.set_autopilot(True, tm.get_port())
# follower.set_autoplot(False)

# Setup spectator view behind follower
spectator = world.get_spectator()

follower_tf = follower.get_transform()
follower_location = follower_tf.location
follower_rotation = follower_tf.rotation

# Get forward vector and move back 8 meters, up 3 meters
back_vector = follower_tf.get_forward_vector() * -8  # move behind
up_vector = carla.Location(z=3)  # camera height

spectator_location = follower_location + back_vector + up_vector
spectator_transform = carla.Transform(spectator_location, follower_rotation)

spectator.set_transform(spectator_transform)

# from idm_controller import IDMController

# Initialize IDM controller (custom logic you define)
idm = IDMController()
controller = FollowerController(world, follower, leader, idm)
# while True:
#     # Get states
#     leader_transform = leader.get_transform()
#     follower_transform = follower.get_transform()
#     leader_velocity = leader.get_velocity()
#     follower_velocity = follower.get_velocity()
#
#     # Compute spacing and speed diwfference
#     dx = leader_transform.location.distance(follower_transform.location)
#     dv = math.sqrt(leader_velocity.x**2 + leader_velocity.y**2 + leader_velocity.z**2) - \
#          math.sqrt(follower_velocity.x**2 + follower_velocity.y**2 + follower_velocity.z**2)
#
#     # IDM step: compute throttle/brake based on dx and dv
#     throttle, brake = idm.step(dx, dv)
#
#     # Control follower
#     control = carla.VehicleControl(throttle=throttle, brake=brake)
#     follower.apply_control(control)
#
#     time.sleep(0.05)  # ~20 Hz update

while True:
    # Get transforms and velocities
    leader_tf = leader.get_transform()
    follower_tf = follower.get_transform()
    leader_vel = leader.get_velocity()
    follower_vel = follower.get_velocity()

    # Compute gap and relative speed
    dx = leader_tf.location.distance(follower_tf.location)
    dv = math.sqrt(leader_vel.x**2 + leader_vel.y**2 + leader_vel.z**2) - \
         math.sqrt(follower_vel.x**2 + follower_vel.y**2 + follower_vel.z**2)

    # IDM control step
    throttle, brake = idm.step(dx, dv)
    follower.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))

    # === Update camera to follow follower ===
    spectator = world.get_spectator()
    back_vector = follower_tf.get_forward_vector() * -8
    camera_location = follower_tf.location + back_vector + carla.Location(z=3)
    camera_tf = carla.Transform(camera_location, follower_tf.rotation)
    spectator.set_transform(camera_tf)

    time.sleep(0.05)  # ~20 Hz



print(0)