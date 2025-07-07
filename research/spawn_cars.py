import carla
import random
import cv2
import numpy as np

from research.load_custom_path import world

# connect to the simulator
client = carla.Client('localhost', 2000)
# load predefine town from carla
client.load_world('Town05')

world = client.get_world()

# extract spawn points
spawn_points = world.get_map().get_spawn_points()
start_point = spawn_points[0]
vehicle_bp = world.get_blueprint_library().filter('*mini*')

vehicle = world.try_spawn_actor(vehicle_bp, start_point)
