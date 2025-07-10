import carla
import pandas as pd
import time
import matplotlib

from research.utils import load_xodr, create_world_from_custom_map

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from settings import ROOT_DIR

class SpeedProfileController:
    def __init__(self, csv_path: str):
        self.df = pd.read_csv(csv_path)
        self._prepare_profile()

    def _prepare_profile(self):
        self.df['speed_mps'] = self.df['Message'] * (1000 / 3600)  # km/h to m/s
        # Drop initial rows where speed is 0
        first_non_zero_idx = self.df[self.df['speed_mps'] > 0].index[0]
        self.df = self.df.iloc[first_non_zero_idx:].reset_index(drop=True)

        # Normalize time to start from zero
        self.df['time_rel'] = self.df['Time'] - self.df['Time'].iloc[0]

    def get_speed_at(self, sim_time: float) -> float:
        return np.interp(sim_time, self.df['time_rel'], self.df['speed_mps'])


class SimulationLogger:
    def __init__(self):
        self.records = []

    def log(self, timestamp: float, location: carla.Location, velocity: carla.Vector3D):
        self.records.append({
            'time': timestamp,
            'x': location.x,
            'y': location.y,
            'speed': np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        })

    def save_plot(self):
        df = pd.DataFrame(self.records)

        # # Plot 1: Trajectory (X-Y)
        # plt.figure(figsize=(8, 6))
        # plt.plot(df['x'], df['y'], label='Trajectory')
        # plt.xlabel('X position (m)')
        # plt.ylabel('Y position (m)')
        # plt.title('Vehicle Trajectory')
        # plt.grid()
        # plt.legend()
        # plt.savefig(f"{ROOT_DIR}/Reports/real_simu_leader_trajectory.png")
        # print("Saved: real_simu_leader_trajectory.png")
        # plt.close()

        # Plot 2: Speed vs Time
        plt.figure(figsize=(8, 6))
        plt.plot(df['time'], df['speed'], label='Speed', color='orange')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.title('Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f"{ROOT_DIR}/Reports/real_simu_leader_speed_vs_time.png")
        print("Saved: real_simu_leader_speed_vs_time.png")
        plt.close()


class CarlaSimulator:
    def __init__(self, csv_path: str, custom_map_path: str):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.load_custom_map(self.client, custom_map_path)
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        self.vehicle = None
        self.spectator = self.world.get_spectator()

        self.controller = SpeedProfileController(csv_path)
        self.logger = SimulationLogger()

    def load_custom_map(self,client, custom_map_path):
        xodr_content = load_xodr(custom_map_path)
        world = create_world_from_custom_map(client, xodr_content)
        return world


    def setup_simulation(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def spawn_vehicle(self):
        spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        vehicle_bp = self.bp_lib.find('vehicle.lincoln.mkz_2020')
        spawn_point.location.x = spawn_point.location.x-500
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

    def get_direction_vector_to_next_waypoint(self, distance_ahead=1.0):
        current_location = self.vehicle.get_location()
        current_wp = self.map.get_waypoint(current_location, project_to_road=True, lane_type=carla.LaneType.Driving)

        next_wp_list = current_wp.next(distance_ahead)
        if not next_wp_list:
            return None  # No more waypoints ahead

        next_wp = next_wp_list[0]
        direction = next_wp.transform.location - current_location
        return direction.make_unit_vector(), next_wp

    def get_direction_and_next_waypoint(self, distance_ahead=2.0):
        current_location = self.vehicle.get_location()
        current_wp = self.map.get_waypoint(current_location, project_to_road=True, lane_type=carla.LaneType.Driving)

        # Try finding the next waypoint; check all possibilities
        try:
            next_wp_list = current_wp.next(distance_ahead)
            if not next_wp_list:
                return None, None  # Gracefully return if no next point found

            next_wp = next_wp_list[0]
            direction = next_wp.transform.location - current_location

            # Check if direction vector is non-zero
            if direction.length() == 0:
                return None, None

            return direction.make_unit_vector(), next_wp

        except RuntimeError as e:
            print(f"Waypoint error: {e}")
            return None, None

    def run(self):
        self.setup_simulation()
        self.spawn_vehicle()
        # 2. Tick world once to finalize registration
        self.world.tick()
        # Warm-up ticks after spawn
        # for _ in range(3):
        #     self.world.tick()

        # 3. Confirm vehicle is spawned and located on road
        location = self.vehicle.get_location()
        waypoint = self.map.get_waypoint(location, project_to_road=True)

        if waypoint is None:
            raise RuntimeError("Vehicle did not spawn on a valid road location.")

        if not self.vehicle:
            print("Vehicle spawn failed.")
            return

        print("Starting simulation with waypoint-following and speed profile...")
        start_time = time.time()
        try:
            while True:
                if self.vehicle:
                    sim_time = time.time() - start_time
                    target_speed = self.controller.get_speed_at(sim_time)

                    direction_vector, next_wp = self.get_direction_vector_to_next_waypoint()
                    # result = self.get_direction_and_next_waypoint()
                    if direction_vector is None:
                        print("No next waypoint. Ending simulation.")
                        break


                    # Set velocity in direction of next waypoint
                    velocity = carla.Vector3D(
                        direction_vector.x * target_speed,
                        direction_vector.y * target_speed,
                        direction_vector.z * target_speed
                    )
                    self.vehicle.set_target_velocity(velocity)

                    # 1. Move in direction of next waypoint
                    # velocity = carla.Vector3D(
                    #     direction_vector.x * target_speed,
                    #     direction_vector.y * target_speed,
                    #     direction_vector.z * target_speed
                    # )
                    # self.vehicle.set_target_velocity(velocity)
                    #
                    # 2. Align vehicle yaw to match next waypoint heading
                    transform = self.vehicle.get_transform()
                    transform.rotation = next_wp.transform.rotation  # Apply next waypoint yaw
                    self.vehicle.set_transform(transform)

                    # Logging
                    print(f"Current location: {self.vehicle.get_location()}, speed: {target_speed:.2f} m/s")

                    self.logger.log(sim_time, self.vehicle.get_location(), self.vehicle.get_velocity())

                    # Update spectator
                    transform = carla.Transform(
                        self.vehicle.get_transform().transform(carla.Location(x=-6, z=2.5)),
                        self.vehicle.get_transform().rotation
                    )
                    self.spectator.set_transform(transform)

                    self.world.tick()

        except KeyboardInterrupt:
            print("Simulation interrupted.")

        finally:
            self.cleanup()

    def cleanup(self):
        if self.vehicle is not None:
            self.vehicle.destroy()
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        self.logger.save_plot()
        print("Simulation complete. Trajectory plotted.")


if __name__ == '__main__':
    custom_map_path = f'{ROOT_DIR}/routes/road_with_object.xodr'
    sim = CarlaSimulator(f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv',custom_map_path)
    sim.run()
