import time

import carla
import random
import numpy as np

from idm_controller import IDMController
from follower_vehicle import FollowerVehicle
from fs_controller import FollowerStopperController
from speed_profiler import SpeedProfileController
from utils import create_world_from_custom_map
from settings import ROOT_DIR
from sim_logger import SimulationLogger
from utils import load_xodr

# Lane selection options for your road:
LANE_OPTIONS = {
    'center': 0.0,  # Road centerline (between opposing traffic)
    'right_inner': 1.5,  # Right driving lane (normal traffic)
    'right_outer': 4.5,  # Right outer lane
    'left_inner': -1.5,  # Left driving lane (opposing traffic)
    'left_outer': -4.5  # Left outer lane (opposing traffic)
}


class CarlaSimulator:
    def __init__(self, csv_path, custom_map_path, controller_name, reference_speed, sampling_frequency, num_ice_followers, switch_time=None):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.load_custom_map(self.client, custom_map_path)
        # self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        self.controller_type = controller_name
        self.switch_time = switch_time

        self.csv_path = csv_path
        self.sampling_frequency = sampling_frequency
        self.speed_controller = SpeedProfileController(self.csv_path, self.sampling_frequency)
        self.num_ice_followers = num_ice_followers

        self.leader = None
        self.followers = []
        self.spectator = self.world.get_spectator()
        self.reference_speed = reference_speed

        self.logger = SimulationLogger(self.controller_type, self.num_ice_followers, self.reference_speed, self.sampling_frequency, self.switch_time)
        self.leader_speed_buffer = []


    def load_custom_map(self,client, custom_map_path):
        xodr_content = load_xodr(custom_map_path)
        world = create_world_from_custom_map(client, xodr_content)
        return world

    def setup_simulation(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.02
        self.world.apply_settings(settings)

    def spawn_vehicle(self, blueprint_name, spawn_transform):
        bp = self.bp_lib.find(blueprint_name)
        vehicle = self.world.try_spawn_actor(bp, spawn_transform)
        return vehicle

    def spawn_vehicle(self, blueprint_name, spawn_transform):
        bp = self.bp_lib.find(blueprint_name)
        vehicle = self.world.try_spawn_actor(bp, spawn_transform)
        return vehicle

    def get_vehicle_length(self,vehicle):
        # Assuming 'vehicle' is a CARLA actor object
        vehicle_extent = vehicle.bounding_box.extent
        vehicle_length = vehicle_extent.x * 2
        # print(f"Vehicle length: {vehicle_length} meters")
        return vehicle_length

    def spawn_leader_and_followers(self):
        spawn_points = self.map.get_spawn_points()
        base_spawn = random.choice(spawn_points)

        # Choose a specific lane for the entire platoon
        # For your road: Y=0 (center), Y=1.5 (right lane), Y=-1.5 (left lane)
        PLATOON_LANE_Y = 1.5  # Right driving lane (adjust as needed)

        # Spawn Leader in chosen lane
        leader_bp = 'vehicle.lincoln.mkz_2020'
        leader_spawn_location = carla.Location(
            x=base_spawn.location.x - 500,  # Start position
            y=PLATOON_LANE_Y,  # Lane position
            z=0.5  # Slight elevation
        )
        leader_transform = carla.Transform(leader_spawn_location, base_spawn.rotation)
        self.leader = self.spawn_vehicle(leader_bp, leader_transform)

        if not self.leader:
            raise RuntimeError("Failed to spawn leader vehicle.")

        leader_length = self.get_vehicle_length(self.leader)
        print(f"Leader spawned in lane Y={PLATOON_LANE_Y}, length: {leader_length}m")

        # Spawn Followers directly behind leader in same lane
        idm_controller = IDMController()
        fs_controller = FollowerStopperController()

        previous_vehicle = self.leader
        cumulative_distance = 0

        for i in range(self.num_ice_followers + 1):
            # Calculate spacing based on vehicle lengths + desired gap
            if i == 0:
                vehicle_length = leader_length
            else:
                prev_follower_length = self.get_vehicle_length(self.followers[i - 1].vehicle)
                vehicle_length = prev_follower_length

            desired_gap = 8.0  # 8 meter gap between vehicles
            spacing = vehicle_length + desired_gap
            cumulative_distance += spacing

            # Spawn follower directly behind in same lane
            follower_location = carla.Location(
                x=leader_spawn_location.x + cumulative_distance,  # Behind leader
                y=PLATOON_LANE_Y,  # Same lane
                z=0.5  # Same elevation
            )
            follower_transform = carla.Transform(follower_location, base_spawn.rotation)

            follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.tesla.model3'
            vehicle = self.spawn_vehicle(follower_bp, follower_transform)
            if not vehicle:
                raise RuntimeError(f"Failed to spawn follower {i}.")

            follower_length = self.get_vehicle_length(vehicle)
            print(
                f"Follower {i} spawned at X={follower_location.x:.1f}, Y={PLATOON_LANE_Y}, length: {follower_length}m")

            # Each follower follows the previous vehicle (not the leader)
            follower = FollowerVehicle(vehicle, self.map, previous_vehicle,
                                       self.reference_speed, idm_controller, fs_controller)
            self.followers.append(follower)
            previous_vehicle = vehicle

    # Modified position constraint to maintain lane discipline
    def apply_lane_constraint(self, vehicle, target_lane_y=1.5):
        """Keep vehicle in specified lane"""
        current_pos = vehicle.get_location()
        y_deviation = abs(current_pos.y - target_lane_y)

        # Only correct if significantly off-lane (more than 0.3m)
        if y_deviation > 0.3:
            transform = vehicle.get_transform()
            transform.location.y = target_lane_y
            transform.location.z = 0.5
            vehicle.set_transform(transform)
            return True
        return False

    # Updated run_synchronously with lane constraint
    def run_synchronously(self, simulation_start_time, simulation_end_time):
        self.setup_simulation()
        self.spawn_leader_and_followers()

        PLATOON_LANE_Y = 1.5  # Same lane Y-coordinate as spawn

        print(f"Starting platoon simulation in lane Y={PLATOON_LANE_Y}...")

        speed_df = self.speed_controller.df
        min_steps = int(simulation_start_time * self.sampling_frequency)
        max_steps = int(simulation_end_time * self.sampling_frequency)

        try:
            for i in range(min_steps, max_steps):
                if i >= len(speed_df):
                    break

                sim_time = speed_df.loc[i, 'time_rel']
                target_speed = speed_df.loc[i, 'speed_mps']
                self.leader_speed_buffer.append(target_speed)

                # --- Leader Control ---
                current_speed = self.leader.get_velocity()
                current_speed_mps = np.sqrt(current_speed.x ** 2 + current_speed.y ** 2)
                speed_error = target_speed - current_speed_mps

                # Keep leader in lane
                leader_corrected = self.apply_lane_constraint(self.leader, PLATOON_LANE_Y)
                if leader_corrected:
                    print(f"Leader corrected to lane Y={PLATOON_LANE_Y}")

                throttle = np.clip(speed_error * 0.5, 0.0, 1.0)
                brake = 0.0 if speed_error >= -0.5 else np.clip(-speed_error * 0.5, 0.0, 1.0)
                if brake > 0:
                    throttle = 0.0

                # Apply straight-line control (no steering)
                control = carla.VehicleControl()
                control.throttle = throttle
                control.brake = brake
                control.steer = 0.0
                self.leader.apply_control(control)

                self.logger.log(sim_time, 'leader', self.leader.get_location(), target_speed,
                                round(self.leader.get_acceleration().x, 3))

                # --- Followers Control ---
                for j, follower in enumerate(self.followers):
                    # Keep follower in same lane
                    follower_corrected = self.apply_lane_constraint(follower.vehicle, PLATOON_LANE_Y)
                    if follower_corrected:
                        print(f"Follower {j} corrected to lane Y={PLATOON_LANE_Y}")

                    # Verify following relationship
                    leader_pos = follower.leader.get_location()
                    follower_pos = follower.vehicle.get_location()

                    # Debug: Check if following the right vehicle
                    if i % 250 == 0:  # Every 5 seconds
                        print(f"Car{j + 1} following vehicle at X={leader_pos.x:.1f}, Y={leader_pos.y:.1f}")
                        print(f"Car{j + 1} current position X={follower_pos.x:.1f}, Y={follower_pos.y:.1f}")

                    # Controller logic
                    if self.switch_time == 0 or (self.switch_time and sim_time >= self.switch_time):
                        # FS Controller
                        latest_speeds = self.leader_speed_buffer[-200:] if len(
                            self.leader_speed_buffer) >= 200 else self.leader_speed_buffer
                        ref_velocity = np.mean(latest_speeds) if latest_speeds else target_speed

                        result = follower.update_fs(ref_velocity)
                        command_velocity, gap, vehicle_location, rel_speed = result

                        self.logger.log(sim_time=sim_time, name=f'car{j + 1}', location=vehicle_location,
                                        velocity=command_velocity, acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)

                        print(
                            f"FS Car{j + 1} - Speed: {command_velocity:.1f} - X: {vehicle_location.x:.1f} - Y: {vehicle_location.y:.2f} - Gap: {gap:.1f}m")

                    else:
                        # IDM Controller
                        result = follower.update_idm()
                        command_velocity, gap, vehicle_location, rel_speed = result

                        self.logger.log(sim_time=sim_time, name=f'car{j + 1}', location=vehicle_location,
                                        velocity=command_velocity, acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=0, rel_speed=rel_speed)

                        print(
                            f"IDM Car{j + 1} - Speed: {command_velocity:.1f} - X: {vehicle_location.x:.1f} - Y: {vehicle_location.y:.2f} - Gap: {gap:.1f}m")

                # Camera follows the platoon
                if self.followers:
                    last_follower = self.followers[-1].vehicle
                    cam_location = last_follower.get_location() + carla.Location(x=-15, y=-3, z=8)
                    cam_rotation = carla.Rotation(pitch=-20, yaw=0, roll=0)
                    self.spectator.set_transform(carla.Transform(cam_location, cam_rotation))

                self.world.tick()

        except KeyboardInterrupt:
            print("Simulation interrupted.")
        finally:
            self.cleanup()

    def get_direction_to_next_wp(self, vehicle, lookahead=1.0):
        current_loc = vehicle.get_location()
        current_wp = self.map.get_waypoint(current_loc, project_to_road=True)
        next_wp_list = current_wp.next(lookahead)
        if not next_wp_list:
            return None, None
        next_wp = next_wp_list[0]
        direction = next_wp.transform.location - current_loc
        return direction.make_unit_vector(), next_wp

    def cleanup(self):
        actors = [self.leader] + [f.vehicle for f in self.followers]
        for a in actors:
            if a is not None:
                a.destroy()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        # Logging and Plots
        self.logger.save()
        # self.logger.plot_trajectories()
        # self.logger.plot_speeds()
        # self.logger.plot_reference_velocity()
        # self.logger.plot_command_velocity()
        # self.logger.plot_relative_velocity()
        # self.logger.plot_gap_vs_time()
        # self.logger.plot_time_space_diagram()
        print("Vehicles destroyed. Simulation ended.")

if __name__ == '__main__':
    controller_name = "FsIdmTesting10"
    # controller_name = "FS_IDM_nomi"
    controller_type = controller_name
    reference_speed = 25
    switch_time = 120
    simulation_start_time = 0.0
    simulation_end_time = 300.0
    # controller_type = "FS_IDM_avg_ref"
    custom_map_path = f'{ROOT_DIR}/routes/road_crosswork.xodr'
    sim = CarlaSimulator(csv_path=f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv',custom_map_path=custom_map_path,controller_name=controller_name, num_ice_followers=6, reference_speed=reference_speed, sampling_frequency=50, switch_time=switch_time)
    sim.run_synchronously(simulation_start_time=simulation_start_time, simulation_end_time=simulation_end_time)