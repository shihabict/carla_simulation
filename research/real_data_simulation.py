import carla
import pandas as pd
import time
import matplotlib.pyplot as plt
import numpy as np
from settings import ROOT_DIR

class SpeedProfileController:
    def __init__(self, csv_path: str):
        self.df = pd.read_csv(csv_path)
        self._prepare_profile()

    def _prepare_profile(self):
        self.df['speed_mps'] = self.df['Message'] * (1000 / 3600)  # km/h to m/s
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
        plt.figure(figsize=(8, 6))
        plt.plot(df['x'], df['y'], label='Trajectory')
        plt.xlabel('X position (m)')
        plt.ylabel('Y position (m)')
        plt.title('Vehicle Trajectory')
        plt.grid()
        plt.legend()
        plt.show()


class CarlaSimulator:
    def __init__(self, csv_path: str):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.bp_lib = self.world.get_blueprint_library()
        self.vehicle = None
        self.spectator = self.world.get_spectator()

        self.controller = SpeedProfileController(csv_path)
        self.logger = SimulationLogger()

    def setup_simulation(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(settings)

    def spawn_vehicle(self):
        spawn_point = self.world.get_map().get_spawn_points()[0]
        vehicle_bp = self.bp_lib.find('vehicle.lincoln.mkz_2020')
        self.vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)

    def run(self):
        self.setup_simulation()
        self.spawn_vehicle()
        if not self.vehicle:
            print("Vehicle spawn failed.")
            return

        print("Starting simulation with speed profile...")
        start_time = time.time()

        try:
            while True:
                sim_time = time.time() - start_time
                target_speed = self.controller.get_speed_at(sim_time)

                # Set forward velocity vector
                forward_vector = self.vehicle.get_transform().get_forward_vector()
                velocity = carla.Vector3D(forward_vector.x * target_speed,
                                          forward_vector.y * target_speed,
                                          forward_vector.z * target_speed)
                self.vehicle.set_target_velocity(velocity)

                self.logger.log(sim_time, self.vehicle.get_location(), self.vehicle.get_velocity())

                # Update spectator view
                transform = carla.Transform(
                    self.vehicle.get_transform().transform(carla.Location(x=-6, z=2.5)),
                    self.vehicle.get_transform().rotation)
                self.spectator.set_transform(transform)

                self.world.tick()

        except KeyboardInterrupt:
            print("Simulation interrupted. Cleaning up...")
            self.cleanup()

    def cleanup(self):
        if self.vehicle is not None:
            self.vehicle.destroy()
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        self.logger.save_plot()
        print("Simulation ended and plot generated.")


if __name__ == '__main__':
    sim = CarlaSimulator(f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv')
    sim.run()
