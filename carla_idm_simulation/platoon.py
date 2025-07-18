import carla
import time
import math

from idm_controller import IDMController
from follower_stopper import FollowerStopperController
from live_plotter import LivePlotter
from data_logger import DataLogger
# from idm_controller import IDMController
from follower_controller_v2 import FollowerController
from leader_controller import LeaderController

# from research.follower_stopper import FollowerStopperController

# ==== CARLA Client Setup ====
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.load_world('Town04')
map = world.get_map()
bp_lib = world.get_blueprint_library()
tm = client.get_trafficmanager()

# ==== Weather Setup ====
world.set_weather(carla.WeatherParameters.CloudyNoon)

# ==== Spawn Points ====
spawn_points = map.get_spawn_points()
leader_index = 5
leader_spawn = spawn_points[leader_index]

# ==== Vehicle Blueprints ====
leader_bp = bp_lib.find('vehicle.tesla.model3')
follower_bp = bp_lib.find('vehicle.audi.tt')

# ==== Spawn Leader ====
leader = world.try_spawn_actor(leader_bp, leader_spawn)

# ==== Spawn N Followers ====
num_followers = 3  # Change this value to spawn more followers
followers = []
follower_controllers = []

previous_vehicle = leader
for i in range(num_followers):
    offset = carla.Location(x=-(i + 1) * 5)
    spawn_location = leader_spawn.transform(offset)
    spawn_transform = carla.Transform(spawn_location, leader_spawn.rotation)

    follower = world.try_spawn_actor(follower_bp, spawn_transform)
    if follower is None:
        print(f"Follower {i+1} failed to spawn.")
        continue

    # Apply physics damping
    physics_control = follower.get_physics_control()
    physics_control.linear_damping = 1
    physics_control.angular_damping = 1.5
    for wheel in physics_control.wheels:
        wheel.damping_rate = 3.0
        wheel.max_steer_angle = 60.0
    follower.apply_physics_control(physics_control)

    idm = IDMController()
    fs = FollowerStopperController(U=15)

    controller = FollowerController(world, follower, previous_vehicle, idm, fs, switch_time=20)

    followers.append(follower)
    follower_controllers.append(controller)

    previous_vehicle = follower

# ==== Generate Custom Path for Leader ====
leader_start_wp = map.get_waypoint(leader.get_location())
leader_path = [leader_start_wp]
for _ in range(500):
    leader_path.append(leader_path[-1].next(2.0)[0])

leader.set_autopilot(True, tm.get_port())
# leader_controller = LeaderController(leader, leader_path)


# ==== Camera Setup behind last follower ====
spectator = world.get_spectator()
last_follower_tf = followers[-1].get_transform()
back_vector = last_follower_tf.get_forward_vector() * -16
camera_location = last_follower_tf.location + back_vector + carla.Location(z=3)
camera_tf = carla.Transform(camera_location, last_follower_tf.rotation)
spectator.set_transform(camera_tf)


# ==== Compute the vehicles label ====
vehicle_labels = ['Leader'] + [f'Follower {i+1}' for i in range(len(followers))]


# ==== Init Plotter and Logger ====
logger = DataLogger(num_followers=len(followers))
plotter = LivePlotter(vehicle_labels)

try:
    while True:
        # leader_controller.run_step()

        for controller in follower_controllers:
            controller.update()

        # === Collect and log leader/follower data ===
        leader_tf = leader.get_transform()
        leader_vel = leader.get_velocity()
        leader_speed = math.sqrt(leader_vel.x**2 + leader_vel.y**2 + leader_vel.z**2)

        # Initialize speed dictionary with leader
        speeds = {'Leader': leader_speed}

        # Optional: log and plot the last follower only
        last_follower = followers[-1]
        last_follower_tf = last_follower.get_transform()
        last_follower_vel = last_follower.get_velocity()

        dx = leader_tf.location.distance(last_follower_tf.location)
        dv = math.sqrt(leader_vel.x**2 + leader_vel.y**2) - math.sqrt(last_follower_vel.x**2 + last_follower_vel.y**2)

        # logger.log_data(leader, last_follower, dx, dv)
        logger.log_data(leader, followers)

        # Add follower speeds to dictionary
        for i, follower in enumerate(followers):
            v = follower.get_velocity()
            follower_speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            speeds[f'Follower {i+1}'] = follower_speed

        # Update plot
        elapsed_time = time.time() - logger.start_time
        plotter.update(elapsed_time, speeds)

        # === Camera: Follow last follower ===
        back_vector = last_follower_tf.get_forward_vector() * -20
        camera_location = last_follower_tf.location + back_vector + carla.Location(z=3)
        camera_tf = carla.Transform(camera_location, last_follower_tf.rotation)
        spectator.set_transform(camera_tf)

except KeyboardInterrupt:
    print("Simulation stopped by user")
finally:
    logger.stop_logging()
    print("Data saved to logs/car_following_data.csv")
    plotter.save_plot('final_speed_plot.png')


