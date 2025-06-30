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
# world = client.load_world('Town05')

with open("../routes/singleLane.xodr", "r") as f:
    xodr_content = f.read()

world = client.generate_opendrive_world(xodr_content)

map = world.get_map()
bp_lib = world.get_blueprint_library()
tm = client.get_trafficmanager()

# ==== Weather Setup ====
world.set_weather(carla.WeatherParameters.CloudyNoon)

# ==== Spawn Points ====
spawn_points = map.get_spawn_points()

spawn_points = world.get_map().get_spawn_points()

if not spawn_points:
    print("⚠️ No spawn points found. Using manual transforms.")
    leader_transform = carla.Transform(carla.Location(x=20, y=5, z=5), carla.Rotation(yaw=0))
    # follower_transform = carla.Transform(carla.Location(x=10, y=5, z=1), carla.Rotation(yaw=0))
else:
    leader_transform = spawn_points[0]
    # follower_transform = spawn_points[1]

# ==== Vehicle Blueprints ====
leader_bp = bp_lib.find('vehicle.tesla.model3')
follower_bp = bp_lib.find('vehicle.audi.tt')

# leader = world.try_spawn_actor(leader_bp, leader_transform)
# follower = world.try_spawn_actor(follower_bp, follower_transform)


# leader_index = 1
# leader_spawn = spawn_points[leader_index]



# ==== Spawn Leader ====
leader = world.try_spawn_actor(leader_bp, leader_transform)

# ==== Spawn N Followers ====
num_followers = 2  # Change this value to spawn more followers
followers = []
follower_controllers = []

previous_vehicle = leader
for i in range(num_followers):
    offset = carla.Location(x=-(i + 1) * 8)
    spawn_location = leader_transform.transform(offset)
    spawn_transform = carla.Transform(spawn_location, leader_transform.rotation)

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

    # idm = IDMController()
    # controller = FollowerController(world, follower, previous_vehicle, idm)

    # controller = FollowerController(world, follower, leader, FollowerStopperController(U=15))

    idm = IDMController()
    fs = FollowerStopperController(U=7.5)

    controller = FollowerController(world, follower, previous_vehicle, idm, fs, switch_time=20)

    followers.append(follower)
    follower_controllers.append(controller)

    previous_vehicle = follower

# ==== Generate Custom Path for Leader ====
leader_start_wp = map.get_waypoint(leader.get_location())
leader_path = [leader_start_wp]
for _ in range(50):
    leader_path.append(leader_path[-1].next(2.0)[0])

leader_controller = LeaderController(leader, leader_path)
#
# leader.set_autopilot(True, tm.get_port())

# ==== Camera Setup behind last follower ====
spectator = world.get_spectator()
last_follower_tf = followers[-1].get_transform()
back_vector = last_follower_tf.get_forward_vector() * -16
camera_location = last_follower_tf.location + back_vector + carla.Location(z=3)
camera_tf = carla.Transform(camera_location, last_follower_tf.rotation)
spectator.set_transform(camera_tf)
if 'smoothed_camera_tf' not in locals():
    smoothed_camera_tf = last_follower_tf


# ==== Compute the vehicles label ====
vehicle_labels = ['Leader'] + [f'Follower {i+1}' for i in range(len(followers))]


# ==== Init Plotter and Logger ====
logger = DataLogger(num_followers=len(followers))
plotter = LivePlotter(vehicle_labels)

try:
    while True:
        leader_wp = map.get_waypoint(leader.get_location())
        if leader_wp.road_id == 1 and leader_wp.s >= 995:
            print("Leader reached the end of the road. Exiting simulation.")
            break
        leader_controller.run_step()

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
        # back_vector = last_follower_tf.get_forward_vector() * -8
        # camera_location = last_follower_tf.location + back_vector + carla.Location(z=3)
        # camera_tf = carla.Transform(camera_location, last_follower_tf.rotation)
        # spectator.set_transform(camera_tf)
        # raw_tf = followers[-1].get_transform()
        # back_vector = raw_tf.get_forward_vector() * -8
        # camera_location = raw_tf.location + back_vector + carla.Location(z=3)
        # target_tf = carla.Transform(camera_location, raw_tf.rotation)
        #
        # # Smooth with exponential moving average
        # alpha = 0.1  # smoothing factor
        #
        # smoothed_loc = smoothed_camera_tf.location
        # smoothed_rot = smoothed_camera_tf.rotation
        #
        # # Interpolate manually
        # smoothed_loc = carla.Location(
        #     x=smoothed_loc.x + alpha * (target_tf.location.x - smoothed_loc.x),
        #     y=smoothed_loc.y + alpha * (target_tf.location.y - smoothed_loc.y),
        #     z=smoothed_loc.z + alpha * (target_tf.location.z - smoothed_loc.z)
        # )
        #
        # smoothed_rot = carla.Rotation(
        #     pitch=smoothed_rot.pitch + alpha * (target_tf.rotation.pitch - smoothed_rot.pitch),
        #     yaw=smoothed_rot.yaw + alpha * (target_tf.rotation.yaw - smoothed_rot.yaw),
        #     roll=smoothed_rot.roll + alpha * (target_tf.rotation.roll - smoothed_rot.roll)
        # )
        #
        # smoothed_camera_tf = carla.Transform(smoothed_loc, smoothed_rot)
        # spectator.set_transform(smoothed_camera_tf)

        # Get the transform of the follower vehicle
        vehicle_tf = followers[-1].get_transform()

        # Driver seat relative offset (adjust if needed)
        # Define driver offset from vehicle origin (forward and upward)
        driver_offset_loc = carla.Location(x=1.7, z=2)

        # Transform to world coordinates
        driver_world_loc = vehicle_tf.transform(driver_offset_loc)

        # Use the vehicle's rotation directly (driver looks forward)
        driver_view_tf = carla.Transform(driver_world_loc, vehicle_tf.rotation)

        # Apply to spectator
        # spectator = world.get_spectator()
        # spectator.set_transform(driver_view_tf)
        # === Camera: Follow last follower ===
        back_vector = last_follower_tf.get_forward_vector() * -20
        camera_location = last_follower_tf.location + back_vector + carla.Location(z=3)
        camera_tf = carla.Transform(camera_location, last_follower_tf.rotation)
        spectator.set_transform(camera_tf)

        # === Debug: Draw labels above vehicles ===
        leader_loc = leader.get_transform().location + carla.Location(z=2)
        world.debug.draw_string(
            leader_loc,
            "Leader",
            draw_shadow=False,
            color=carla.Color(0, 255, 0),  # Bright green
            life_time=0.1
        )

        for i, follower in enumerate(followers):
            label = f"Follower {i + 1}"
            loc = follower.get_transform().location + carla.Location(z=3)
            world.debug.draw_string(
                loc,
                label,
                draw_shadow=True,
                color=carla.Color(255, 255, 0),  # Bright yellow
                life_time=0.1
            )



except KeyboardInterrupt:
    print("Simulation stopped by user")
finally:
    logger.stop_logging()
    print("Data saved to logs/car_following_data.csv")
    plotter.save_plot('final_speed_plot.png')


