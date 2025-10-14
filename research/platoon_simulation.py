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
        # Add this after spawning each vehicle:
        # physics_control = vehicle.get_physics_control()
        # physics_control.use_sweep_wheel_collision = True
        # physics_control.
        # vehicle.apply_physics_control(physics_control)
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

        # Spawn Leader
        leader_bp = 'vehicle.lincoln.mkz_2020'
        base_spawn.location.x -= 500
        # base_spawn.location.y = 0.00
        # base_spawn.rotation.yaw = 10.00
        self.leader = self.spawn_vehicle(leader_bp, base_spawn)


        if not self.leader:
            raise RuntimeError("Failed to spawn leader vehicle.")

        leader_length = self.get_vehicle_length(self.leader)
        print(f" Leader Spawned and Length is {leader_length} ")

        # Spawn Followers (8 meters apart behind the leader)
        idm_controller = IDMController()
        fs_controller = FollowerStopperController()

        previous_vehicle = self.leader
        for i in range(self.num_ice_followers + 1):  # First AV + n ICE
            offset_distance = (i + 1) * 8
            base_spawn.location.y = 0.00
            # base_spawn.rotation.yaw = 10.00
            offset_location = base_spawn.location + carla.Location(x=offset_distance)
            follower_transform = carla.Transform(offset_location, base_spawn.rotation)

            follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.tesla.model3'
            # follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.toyota.prius'
            # follower_bp = 'vehicle.toyota.prius'
            vehicle = self.spawn_vehicle(follower_bp, follower_transform)
            if not vehicle:
                raise RuntimeError(f"Failed to spawn follower {i}.")

            follower_length = self.get_vehicle_length(vehicle)
            print(f"[Follower {i} Spawned and length is {follower_length}]")

            # Attach both controllers

            follower = FollowerVehicle(vehicle, self.map, previous_vehicle, self.reference_speed, idm_controller, fs_controller,)

            self.followers.append(follower)
            previous_vehicle = vehicle
            vehicle.set_transform(follower_transform)



    def compute_lateral_control(self, vehicle, target_y=-1.75):
        current_y = vehicle.get_location().y
        y_error = target_y - current_y
        # Simple proportional control
        steer = np.clip(y_error * 0.1, -0.3, 0.3)  # Limit steering
        return steer

    def run_synchronously(self, simulation_start_time, simulation_end_time):
        self.setup_simulation()
        self.spawn_leader_and_followers()

        print("Starting leader-follower simulation...")

        speed_df = self.speed_controller.df  # Convenience alias
        min_steps = int(simulation_start_time*self.sampling_frequency)
        max_steps = int(simulation_end_time*self.sampling_frequency)
        print(f"Max Steps : {max_steps}")
        try:
            for i in range(min_steps, max_steps):
                # Step-specific timing and speed
                sim_time = speed_df.loc[i, 'time_rel']
                target_speed = speed_df.loc[i, 'speed_mps']
                self.leader_speed_buffer.append(target_speed)

                # --- Get current state and direction ---
                direction_vector, next_wp = self.get_direction_to_next_wp(self.leader)
                if direction_vector is None:
                    print("Leader has no more waypoints.")
                    break

                # leader_transform = self.leader.get_transform()
                # leader_transform.location.y = 0.0
                # leader_transform.rotation.yaw = 0.0  # Add this line to control yaw
                # self.leader.set_transform(leader_transform)

                # --- Compute throttle ---
                current_speed = self.leader.get_velocity()
                current_speed_mps = np.linalg.norm([current_speed.x, current_speed.y])
                speed_error = target_speed - current_speed_mps
                self.leader.get_transform().location.y = 0.0
                # print(f"Leader Position in x axis - {self.leader.get_transform().location.x}")

                throttle = np.clip(speed_error * 0.5, 0.0, 1.0)  # Proportional control
                brake = 0.0
                if speed_error < -0.5:
                    brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
                    throttle = 0.0

                # --- Apply control ---
                control = carla.VehicleControl()
                control.throttle = throttle
                control.brake = brake
                control.steer = 0.0
                if control.steer>0.0:
                    print(f"Leader Steering : {control.steer}")
                self.leader.apply_control(control)

                self.logger.log(sim_time, 'leader', self.leader.get_location(), target_speed, round(self.leader.get_acceleration().x,3))

                # --- Followers control ---
                # previous_leader_vel = velocity
                leader_y_position = self.leader.get_location().y
                for j, follower in enumerate(self.followers):

                    if self.switch_time == 0:
                        latest_leader_speed = self.leader_speed_buffer[-200:]
                        ref_velocity = np.mean(latest_leader_speed)
                        command_velocity, gap, vehicle_location, rel_speed = follower.update_fs(ref_velocity)
                        # gap, leader_speed = follower.compute_gap_and_leader_speed()
                        self.logger.log(sim_time=sim_time, name=f'car{j + 1}', location=vehicle_location,
                                        velocity=command_velocity, acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)
                        print(
                            f"FS - {command_velocity} - position - {vehicle_location.x} - Time {sim_time}")
                    elif self.switch_time and sim_time >= simulation_start_time + self.switch_time:
                        latest_leader_speed = self.leader_speed_buffer[-200:]
                        ref_velocity = np.mean(latest_leader_speed)
                        command_velocity, gap, vehicle_location, rel_speed = follower.update_fs(ref_velocity)
                        gap, leader_speed = follower.compute_gap_and_leader_speed()
                        self.logger.log(sim_time=sim_time, name=f'car{j+1}', location=vehicle_location,
                                        velocity=command_velocity, acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)
                        print(
                            f"FS - {command_velocity} - position - {follower.vehicle.get_location().x} - Time {sim_time}")
                    else:
                        command_velocity, gap, vehicle_location, rel_speed = follower.update_idm()
                        ref_velocity = 0

                        # gap, leader_speed = follower.compute_gap_and_leader_speed()
                        self.logger.log(sim_time=sim_time, name=f'car{j+1}',
                                        location=vehicle_location,
                                        velocity=command_velocity, acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)
                        print(f"IDM - {command_velocity} - position - {vehicle_location.x} - Time {sim_time} - Label - car{j+1}")
                    leader_y_position = follower.vehicle.get_location().y


                # # --- Spectator follows last follower ---
                last_follower = self.followers[-1].vehicle
                cam_transform = last_follower.get_transform().transform(carla.Location(x=-10, z=15))
                self.spectator.set_transform(carla.Transform(cam_transform, last_follower.get_transform().rotation))

                # Sync CARLA to time step
                self.world.tick()
                # time.sleep(0.02)
        except KeyboardInterrupt:
            print("Simulation interrupted.")

        finally:
            # print(f"Value of I {i}")
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
    controller_name = "FsIdmTesting6"
    # controller_name = "FS_IDM_nomi"
    controller_type = controller_name
    reference_speed = 25
    switch_time = 20
    simulation_start_time = 0.0
    simulation_end_time = 500.0
    # controller_type = "FS_IDM_avg_ref"
    # custom_map_path = f'{ROOT_DIR}/routes/road_with_object.xodr'
    custom_map_path = f'{ROOT_DIR}/routes/longRoad.xodr'
    sim = CarlaSimulator(csv_path=f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv',custom_map_path=custom_map_path,controller_name=controller_name, num_ice_followers=6, reference_speed=reference_speed, sampling_frequency=50, switch_time=switch_time)
    sim.run_synchronously(simulation_start_time=simulation_start_time, simulation_end_time=simulation_end_time)