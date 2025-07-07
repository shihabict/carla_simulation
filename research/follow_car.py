import carla
import matplotlib.pyplot as plt
from settings import BASE_DIR
from utils import load_xodr, create_world_from_custom_map, load_speed_profile


def connect_to_carla(host='localhost', port=2000, timeout=10.0):
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def setup_world(client, xodr_path, sync=True, delta_seconds=0.05):
    xodr_content = load_xodr(xodr_path)
    world = create_world_from_custom_map(client, xodr_content)
    # Set synchronous mode for full simulation time control
    if sync:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = delta_seconds
        world.apply_settings(settings)
    return world


def spawn_vehicle(world, bp_name='vehicle.tesla.model3', spawn_point_idx=0, x_offset=-50):
    bp_lib = world.get_blueprint_library()
    bp = bp_lib.find(bp_name)
    spawn_points = world.get_map().get_spawn_points()
    spawn_point = spawn_points[spawn_point_idx]
    spawn_point.location.x += x_offset
    vehicle = world.try_spawn_actor(bp, spawn_point)
    return vehicle


def set_spectator_view(world, spawn_point_idx=1, z_offset=3, x_offset=-40):
    spectator = world.get_spectator()
    spawn_points = world.get_map().get_spawn_points()
    camera_position = spawn_points[spawn_point_idx]
    camera_position.location.z += z_offset
    camera_position.location.x += x_offset
    spectator.set_transform(camera_position)


def follow_fixed_duration(vehicle, target_speed, world, sim_seconds=300, delta_seconds=0.05):
    """Run the simulation for a fixed duration, applying a constant target_speed."""
    n_ticks = int(sim_seconds / delta_seconds)
    trajectory = []
    for i in range(n_ticks):
        # Simple proportional controller to hold target speed
        v = vehicle.get_velocity()
        current_speed = (v.x**2 + v.y**2 + v.z**2)**0.5
        error = target_speed - current_speed
        control = carla.VehicleControl()
        control.throttle = min(max(error * 0.5, 0.0), 1.0)
        control.brake = 0.0 if error >= 0 else min(abs(error) * 0.5, 1.0)
        vehicle.apply_control(control)
        # Record position
        transform = vehicle.get_transform()
        trajectory.append((transform.location.x, transform.location.y))
        world.tick()
    return trajectory


def follow_speed_profile_simtime(vehicle, speed_profile, world, delta_seconds=0.05):
    """Make vehicle follow speed profile using simulation time (ticks). Return trajectory."""
    trajectory = []
    n_steps = len(speed_profile)
    for i in range(n_steps):
        rel_time, target_speed = speed_profile[i]
        # Control logic (simple proportional controller)
        v = vehicle.get_velocity()
        current_speed = (v.x ** 2 + v.y ** 2 + v.z ** 2) ** 0.5
        error = target_speed - current_speed
        control = carla.VehicleControl()
        control.throttle = min(max(error * 0.5, 0.0), 1.0)
        control.brake = 0.0 if error >= 0 else min(abs(error) * 0.5, 1.0)
        vehicle.apply_control(control)
        # Record (x, y)
        transform = vehicle.get_transform()
        trajectory.append((transform.location.x, transform.location.y))
        world.tick()  # Advance the simulation one step (synchronous mode)
    return trajectory


def plot_trajectory(trajectory):
    xs, ys = zip(*trajectory)
    plt.figure(figsize=(8, 6))
    plt.plot(xs, ys, marker='o', linewidth=2)
    plt.title('Leader Vehicle Trajectory')
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.axis('equal')
    plt.grid(True)
    plt.show()


def main():
    csv_path = "../datasets/CAN_Messages_decoded_speed.csv"
    speed_profile = load_speed_profile(csv_path)  # Must return list of (rel_time, speed_mps)

    client = connect_to_carla()
    xodr_path = f"{BASE_DIR}/routes/simple_road.xodr"
    world = setup_world(client, xodr_path, sync=True, delta_seconds=0.05)  # 20Hz sim time

    leader = spawn_vehicle(world)
    set_spectator_view(world)

    # trajectory = follow_speed_profile_simtime(leader, speed_profile, world, delta_seconds=0.05)
    trajectory = follow_fixed_duration(leader, target_speed=15.0, world=world, sim_seconds=50, delta_seconds=0.05)
    plot_trajectory(trajectory)


if __name__ == "__main__":
    main()
