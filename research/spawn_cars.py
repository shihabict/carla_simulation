import carla
import random
import cv2
import numpy as np

# connect to the simulator
client = carla.Client('localhost', 2000)
# load predefine town from carla
client.load_world('Town05')

# extract spawn points
spawn_points = world.get_map().get_spawn_points()
start_point = spawn_points[0]
vehicle_bp = world.get_blueprint_library().filter('*mini*')

vehicle = world.try_spawn_actor(vehicle_bp, start_point)
