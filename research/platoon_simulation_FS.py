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
    def __init__(self, csv_path, custom_map_path, controller_name, reference_speed, sampling_frequency,
                 num_ice_followers, switch_time=None):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.load_custom_map(self.client, custom_map_path)
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

        self.logger = SimulationLogger(self.controller_type, self.num_ice_followers, self.reference_speed,
                                       self.sampling_frequency, self.switch_time)
        self.leader_speed_buffer = []

    def load_custom_map(self, client, custom_map_path):
        xodr_content = load_xodr(custom_map_path)
        world = create_world_from_custom_map(client, xodr_content)
        return world

    def setup_simulation(self):
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 0.02
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

    def spawn_vehicle(self, blueprint_name, spawn_transform):
        bp = self.bp_lib.find(blueprint_name)
        vehicle = self.world.try_spawn_actor(bp, spawn_transform)
        return vehicle

    def spawn_leader_and_followers(self):
        spawn_points = self.map.get_spawn_points()
        base_spawn = random.choice(spawn_points)

        # Spawn Leader
        leader_bp = 'vehicle.lincoln.mkz_2020'
        base_spawn.location.x -= 500
        base_spawn.location.y = 0.00
        self.leader = self.spawn_vehicle(leader_bp, base_spawn)
        if not self.leader:
            raise RuntimeError("Failed to spawn leader vehicle.")
        print("[Leader Spawned]")

        # Spawn Followers (8 meters apart behind the leader)
        idm_controller = IDMController()
        fs_controller = FollowerStopperController()

        previous_vehicle = self.leader
        for i in range(self.num_ice_followers + 1):  # First AV + n ICE
            offset_distance = (i + 1) * 8.6
            base_spawn.location.y = 0.00
            offset_location = base_spawn.location + carla.Location(x=offset_distance)
            follower_transform = carla.Transform(offset_location, base_spawn.rotation)

            follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.tesla.model3'
            vehicle = self.spawn_vehicle(follower_bp, follower_transform)
            if not vehicle:
                raise RuntimeError(f"Failed to spawn follower {i}.")
            print(f"[Follower {i} Spawned]")

            # Attach both controllers

            follower = FollowerVehicle(vehicle, self.map, previous_vehicle, self.reference_speed, idm_controller,
                                       fs_controller, )

            self.followers.append(follower)
            previous_vehicle = vehicle

    def get_steering_control(self, vehicle, target_speed):
        """Calculate steering to keep vehicle on road centerline"""
        current_loc = vehicle.get_location()
        current_wp = self.map.get_waypoint(current_loc, project_to_road=True)

        # Get next waypoint for direction
        next_wps = current_wp.next(2.0)  # Look ahead 2 meters
        if not next_wps:
            return 0.0

        next_wp = next_wps[0]

        # Calculate steering angle to follow the road
        vehicle_transform = vehicle.get_transform()
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)

        # Vector from vehicle to next waypoint
        target_location = next_wp.transform.location
        vehicle_location = vehicle_transform.location

        dx = target_location.x - vehicle_location.x
        dy = target_location.y - vehicle_location.y
        target_yaw = np.arctan2(dy, dx)

        # Calculate steering error
        yaw_error = target_yaw - vehicle_yaw

        # Normalize to [-pi, pi]
        while yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        while yaw_error < -np.pi:
            yaw_error += 2 * np.pi

        # Convert to steering command (scale based on speed)
        speed_factor = max(0.1, min(1.0, target_speed / 10.0))  # Reduce steering at higher speeds
        steering = np.clip(yaw_error * 0.5 * speed_factor, -0.3, 0.3)

        return steering

    def get_direction_to_next_wp(self, vehicle, lookahead=1.0):
        current_loc = vehicle.get_location()
        current_wp = self.map.get_waypoint(current_loc, project_to_road=True)
        next_wp_list = current_wp.next(lookahead)
        if not next_wp_list:
            return None, None
        next_wp = next_wp_list[0]
        direction = next_wp.transform.location - current_loc
        return direction.make_unit_vector(), next_wp

    def run_synchronously(self, simulation_start_time, simulation_end_time):
        self.setup_simulation()
        self.spawn_leader_and_followers()
        print(f"Leader ID: {self.leader.id}, Location: {self.leader.get_location()}")
        print("Starting leader-follower simulation...")

        speed_df = self.speed_controller.df
        min_steps = int(simulation_start_time * self.sampling_frequency)
        max_steps = int(simulation_end_time * self.sampling_frequency)
        print(f"Max Steps: {max_steps}")

        try:
            for i in range(min_steps, max_steps):
                sim_time = speed_df.loc[i, 'time_rel']
                delta_t = 0.02
                target_speed = speed_df.loc[i, 'speed_mps']
                self.leader_speed_buffer.append(target_speed)

                # --- Get current state and direction ---
                direction_vector, next_wp = self.get_direction_to_next_wp(self.leader)
                if direction_vector is None:
                    print("Leader has no more waypoints.")
                    break

                # current_transform = self.leader.get_transform()
                #
                # current_transform.location.y = 0.0
                # current_transform.location.z = 0.0
                # self.leader.set_transform(current_transform)

                # --- Leader Control ---
                current_speed = self.leader.get_velocity()
                current_speed_mps = np.linalg.norm([current_speed.x, current_speed.y])
                speed_error = target_speed - current_speed_mps

                # Speed control
                throttle = np.clip(speed_error * 0.5, 0.0, 1.0)
                brake = 0.0
                if speed_error < -0.5:
                    brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
                    throttle = 0.0

                # Steering control to keep on road
                steering = self.get_steering_control(self.leader, target_speed)

                # Apply control to leader
                control = carla.VehicleControl()
                control.throttle = throttle
                control.brake = brake
                control.steer = steering
                self.leader.apply_control(control)

                # Log leader data
                self.logger.log(sim_time, 'leader', self.leader.get_location(), target_speed,
                                round(self.leader.get_acceleration().x, 3))

                # --- Followers Control ---
                for j, follower in enumerate(self.followers):
                    label = f'car{j + 1}'
                    # follower_current_transform = follower.vehicle.get_transform()
                    # follower_current_transform.location.y = 0.0
                    # follower_current_transform.location.z = 0.0
                    # follower.vehicle.set_transform(follower_current_transform)

                    # Controller switching logic
                    if self.switch_time == 0:
                        latest_leader_speed = self.leader_speed_buffer[-200:]
                        ref_velocity = np.mean(latest_leader_speed)
                        command_velocity, rel_speed, quadratic_region = follower.update_fs(ref_velocity)
                        gap, leader_speed = follower.compute_gap_and_leader_speed()
                        self.logger.log(sim_time=sim_time, name=label,
                                        location=follower.vehicle.get_location(),
                                        velocity=command_velocity,
                                        acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)
                        print(f"FS - {command_velocity:.2f} - Time: {sim_time:.2f}")

                    elif self.switch_time and sim_time >= simulation_start_time + self.switch_time:
                        latest_leader_speed = self.leader_speed_buffer[-200:]
                        ref_velocity = np.mean(latest_leader_speed)
                        command_velocity, rel_speed, quadratic_region = follower.update_fs(ref_velocity)
                        gap, leader_speed = follower.compute_gap_and_leader_speed()
                        self.logger.log(sim_time=sim_time, name=label,
                                        location=follower.vehicle.get_location(),
                                        velocity=command_velocity,
                                        acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)
                        print(f"FS - {command_velocity:.2f} - Time: {sim_time:.2f}")

                    else:
                        command_velocity, rel_speed = follower.update_idm()
                        ref_velocity = 0
                        gap, leader_speed = follower.compute_gap_and_leader_speed()
                        self.logger.log(sim_time=sim_time, name=label,
                                        location=follower.vehicle.get_location(),
                                        velocity=command_velocity,
                                        acceleration=follower.vehicle.get_acceleration().x,
                                        gap=gap, ref_speed=ref_velocity, rel_speed=rel_speed)
                        print(f"IDM - {command_velocity:.2f} - Time: {sim_time:.2f}")

                # --- Camera follows last follower ---
                if self.followers:
                    last_follower = self.followers[-1].vehicle
                    cam_transform = last_follower.get_transform()
                    cam_location = cam_transform.location + carla.Location(x=-10, z=5)
                    self.spectator.set_transform(carla.Transform(cam_location, cam_transform.rotation))

                # Advance simulation
                self.world.tick()

        except KeyboardInterrupt:
            print("Simulation interrupted.")
        finally:
            self.cleanup()

    def cleanup(self):
        actors = [self.leader] + [f.vehicle for f in self.followers]
        for actor in actors:
            if actor is not None:
                actor.destroy()

        settings = self.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)

        # Save logs
        self.logger.save()
        print("Vehicles destroyed. Simulation ended.")


if __name__ == '__main__':
    controller_name = "FsIdmTesting2"
    reference_speed = 25
    switch_time = 10.0
    simulation_start_time = 0.0
    simulation_end_time = 300.0
    custom_map_path = f'{ROOT_DIR}/routes/road_with_object.xodr'

    sim = CarlaSimulator(
        csv_path=f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv',
        custom_map_path=custom_map_path,
        controller_name=controller_name,
        num_ice_followers=6,
        reference_speed=reference_speed,
        sampling_frequency=50,
        switch_time=switch_time
    )
    sim.run_synchronously(simulation_start_time=simulation_start_time,
                          simulation_end_time=simulation_end_time)