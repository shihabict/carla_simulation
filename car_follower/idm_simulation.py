import carla
import random
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import threading

# ===========================
# === IDM PARAMETERS ========
# ===========================
IDM_PARAMS = {
    "v0": 12.5,      # desired velocity (m/s)
    "T": 1.5,        # safe time headway (s)
    "a": 1.5,        # max acceleration (m/s^2)
    "b": 2.0,        # comfortable deceleration (m/s^2)
    "delta": 4.0,    # acceleration exponent
    "s0": 2.0        # minimum spacing (m)
}

# ===========================
# === GLOBAL VARIABLES ======
# ===========================
SIM_DT = 0.05
SIM_DURATION = 120  # Max simulation duration as backup (seconds)
LEADER_SPEED_LIMIT = 12.5
FOLLOWER_SPACING = [10.0, 20.0]
RANDOM_ACC_INTERVAL = 10.0
LEADER_ACC_RANGE = (-2.0, 2.0)

# ===========================
# === IDM CONTROLLER ========
# ===========================
def idm_acceleration(v, s, delta_v):
    """Compute IDM acceleration."""
    p = IDM_PARAMS
    s_star = p["s0"] + max(0.0, v * p["T"] + (v * delta_v) / (2 * np.sqrt(p["a"] * p["b"])))
    acc = p["a"] * (1 - (v / p["v0"]) ** p["delta"] - (s_star / max(s, 0.1)) ** 2)
    return acc

# ===========================
# === DATA LOGGER ===========
# ===========================
class Logger:
    def __init__(self):
        self.data = []

    def log(self, timestamp, vehicle, label):
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.data.append([timestamp, label, transform.location.x, transform.location.y, speed])

    def save(self, filename="carla_idm_log.csv"):
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "vehicle", "x", "y", "speed"])
            writer.writerows(self.data)

# ===========================
# === PLOTTING ==============
# ===========================
class LivePlotter:
    def __init__(self, labels):
        self.fig, self.ax = plt.subplots()
        self.lines = {label: self.ax.plot([], [], label=label)[0] for label in labels}
        self.ax.set_xlim(0, SIM_DURATION)
        self.ax.set_ylim(0, IDM_PARAMS["v0"] + 5)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.legend()
        self.data = {label: {"x": [], "y": []} for label in labels}
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=200)

    def update(self, timestamp, label, speed):
        self.data[label]["x"].append(timestamp)
        self.data[label]["y"].append(speed)

    def update_plot(self, frame):
        for label, line in self.lines.items():
            line.set_data(self.data[label]["x"], self.data[label]["y"])
        self.ax.relim()
        self.ax.autoscale_view()

    def show(self):
        plt.show()

# ===========================
# === MAIN SIMULATION =======
# ===========================
def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.load_world("Town05")
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]

    spawn_points = world.get_map().get_spawn_points()
    leader = world.spawn_actor(vehicle_bp, spawn_points[0])
    followers = []

    # Spawn followers with fixed spacing
    for spacing in FOLLOWER_SPACING:
        spawn = carla.Transform(spawn_points[0].location - carla.Location(x=spacing), spawn_points[0].rotation)
        followers.append(world.spawn_actor(vehicle_bp, spawn))

    all_vehicles = [leader] + followers
    for v in all_vehicles:
        v.set_autopilot(False)

    logger = Logger()
    plotter = LivePlotter(["leader", "follower1", "follower2"])

    # Initial conditions
    leader_velocity = 0.0
    follower_velocities = [0.0, 0.0]
    last_acc_change = time.time()
    acc_command = 1.0
    start_x = leader.get_transform().location.x

    def plot_thread():
        plotter.show()

    threading.Thread(target=plot_thread, daemon=True).start()

    try:
        sim_start = time.time()
        while True:
            timestamp = time.time() - sim_start

            # Update leader acceleration every 10 seconds
            if time.time() - last_acc_change > RANDOM_ACC_INTERVAL:
                acc_command = random.uniform(*LEADER_ACC_RANGE)
                last_acc_change = time.time()

            # Apply leader acceleration
            leader_velocity += acc_command * SIM_DT
            leader_velocity = np.clip(leader_velocity, 0, LEADER_SPEED_LIMIT)
            leader.set_target_velocity(carla.Vector3D(x=leader_velocity, y=0, z=0))

            # Apply IDM to each follower
            for i, follower in enumerate(followers):
                lead = leader if i == 0 else followers[i - 1]
                s = lead.get_transform().location.x - follower.get_transform().location.x
                v = follower_velocities[i]
                delta_v = v - np.sqrt(lead.get_velocity().x**2)
                acc = idm_acceleration(v, s, delta_v)
                follower_velocities[i] += acc * SIM_DT
                follower_velocities[i] = np.clip(follower_velocities[i], 0, IDM_PARAMS["v0"] + 5)
                follower.set_target_velocity(carla.Vector3D(x=follower_velocities[i], y=0, z=0))

            # Logging
            logger.log(timestamp, leader, "leader")
            plotter.update(timestamp, "leader", leader_velocity)
            for i, follower in enumerate(followers):
                logger.log(timestamp, follower, f"follower{i+1}")
                plotter.update(timestamp, f"follower{i+1}", follower_velocities[i])

            # Check if leader has driven 1000 meters
            current_x = leader.get_transform().location.x
            if current_x - start_x >= 1000.0 or timestamp >= SIM_DURATION:
                break

            time.sleep(SIM_DT)
    finally:
        logger.save()
        print('Simulation complete. Data saved.')
        for v in all_vehicles:
            v.destroy()

if __name__ == '__main__':

    main()
