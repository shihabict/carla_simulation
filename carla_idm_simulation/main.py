import carla
import random
import time
import csv
import math
import os
import pygame

from idm_controller import IDMController

# ==== Setup Logs Directory ====
os.makedirs("logs", exist_ok=True)

# ==== CARLA Client Setup ====
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town01')
bp_lib = world.get_blueprint_library()
tm = client.get_trafficmanager()

# ==== Weather Setup ====
world.set_weather(carla.WeatherParameters.ClearNoon)

# ==== Spawn Points ====
spawn_points = world.get_map().get_spawn_points()

# ==== Select Better-Aligned Spawn Locations ====
# leader_spawn = spawn_points[8]
# follower_spawn = spawn_points[10]

leader_index = 8
follower_index = leader_index + 2  # usually right behind in same lane
leader_spawn = spawn_points[leader_index]
follower_spawn = spawn_points[follower_index]

# ==== Vehicle Blueprints ====
leader_bp = bp_lib.find('vehicle.tesla.model3')
follower_bp = bp_lib.find('vehicle.audi.etron')

# ==== Spawn Vehicles ====

# ==== Spawn Vehicles ====
leader = world.try_spawn_actor(leader_bp, leader_spawn)
follower = world.try_spawn_actor(follower_bp, follower_spawn)

if not leader:
    raise Exception("Failed to spawn leader vehicle.")

if not follower:
    raise Exception("Failed to spawn follower vehicle. Try a different spawn point.")

# leader = world.try_spawn_actor(leader_bp, leader_spawn)
# follower = world.try_spawn_actor(follower_bp, follower_spawn)
#
# if not leader or not follower:
#     raise Exception("Failed to spawn vehicles. Try different spawn points.")
#
# print(f"Leader spawned at: {leader.get_transform().location}")
# print(f"Follower spawned at: {follower.get_transform().location}")

# ==== Set Autopilot for Leader ====
leader.set_autopilot(True, tm.get_port())

# ==== Spectator Camera Behind Follower ====
spectator = world.get_spectator()
cam_offset = carla.Location(x=-6, z=2.5)

# ==== Logging Setup ====
log_file = open("logs/car_following_log.csv", mode='w', newline='')
csv_writer = csv.writer(log_file)
csv_writer.writerow(["time", "leader_x", "leader_y", "follower_x", "follower_y",
                     "leader_speed", "follower_speed", "gap", "rel_vel", "accel"])

# ==== IDM Controller ====
idm = IDMController(v0=20.0, T=1.5, a_max=2.0, b=2.0, delta=4, s0=2.0)

# ==== Delay Simulation Start Until Leader Moves ====
print("Waiting for leader to start moving...")
for _ in range(10):
    world.tick()

    # ==== Simulation Loop ====
    simulation_duration = 50  # seconds
    start_time = time.time()

    pygame.init()
    clock = pygame.time.Clock()

    try:
        while time.time() - start_time < simulation_duration:
            world.tick()

            if not follower.is_alive or not leader.is_alive:
                print("One of the vehicles was destroyed.")
                break

            # Get transforms and velocities
            transform_leader = leader.get_transform()
            transform_follower = follower.get_transform()

            velocity_leader = leader.get_velocity()
            velocity_follower = follower.get_velocity()

            # Compute speeds
            speed_leader = math.sqrt(velocity_leader.x**2 + velocity_leader.y**2 + velocity_leader.z**2)
            speed_follower = math.sqrt(velocity_follower.x**2 + velocity_follower.y**2 + velocity_follower.z**2)

            # Compute gap and relative velocity
            gap = transform_leader.location.distance(transform_follower.location)
            gap = max(gap, 0.1)
            rel_vel = speed_follower - speed_leader

            # IDM acceleration
            acceleration = idm.calculate_acceleration(gap, speed_follower, speed_leader)

            # Convert acceleration to control
            control = carla.VehicleControl()
            if acceleration >= 0:
                control.throttle = min(acceleration / idm.a_max, 1.0)
                control.brake = 0.0
            else:
                control.throttle = 0.0
                control.brake = min(abs(acceleration) / idm.b, 1.0)

            follower.apply_control(control)

            # Update spectator camera
            ego_transform = follower.get_transform()
            camera_rotation = carla.Rotation(pitch=-10, yaw=ego_transform.rotation.yaw)
            spectator_transform = carla.Transform(ego_transform.transform(cam_offset), camera_rotation)
            spectator.set_transform(spectator_transform)

            # Log data
            csv_writer.writerow([
                round(time.time() - start_time, 2),
                transform_leader.location.x, transform_leader.location.y,
                transform_follower.location.x, transform_follower.location.y,
                round(speed_leader, 2), round(speed_follower, 2),
                round(gap, 2), round(rel_vel, 2), round(acceleration, 2)
            ])

            # Limit frame rate
            clock.tick(20)

    finally:
        # Cleanup
        leader.destroy()
        follower.destroy()
        log_file.close()
        print("Actors destroyed. Log saved.")
