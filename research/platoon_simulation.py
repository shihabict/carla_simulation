from gc import enable

import carla
import random
import time

import numpy as np
from carla.libcarla import Vector3D

from idm_controller import IDMController
from follower_vehicle import FollowerVehicle
from fs_controller import FollowerStopperController
# from research.spectator_view import vehicle
from speed_profiler import SpeedProfileController
# from research.real_data_simulation import SpeedProfileController
from utils import create_world_from_custom_map
from settings import ROOT_DIR
from sim_logger import SimulationLogger
from utils import load_xodr


class CarlaSimulator:
    def __init__(self, csv_path, custom_map_path, controller_name, reference_speed, num_ice_followers=3):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.load_custom_map(self.client, custom_map_path)
        # self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        self.controller_type = controller_name

        self.csv_path = csv_path
        self.speed_controller = SpeedProfileController(self.csv_path)
        self.num_ice_followers = num_ice_followers

        self.leader = None
        self.followers = []
        self.spectator = self.world.get_spectator()
        self.reference_speed = reference_speed
        self.logger = SimulationLogger(self.controller_type, self.num_ice_followers, self.reference_speed)

    def load_custom_map(self,client, custom_map_path):
        xodr_content = load_xodr(custom_map_path)
        world = create_world_from_custom_map(client, xodr_content)
        return world

    def setup_simulation(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        # settings.synchronous_mode = False
        # settings.fixed_delta_seconds = 0.05
        # settings.fixed_delta_seconds = 0.01
        # self.delta_t = settings.fixed_delta_seconds
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
        base_spawn.location.x = base_spawn.location.x - 500
        self.leader = self.spawn_vehicle(leader_bp, base_spawn)
        if not self.leader:
            raise RuntimeError("Failed to spawn leader vehicle.")
        print("[Leader Spawned]")

        # Spawn Followers (8 meters apart behind the leader)
        previous_vehicle = self.leader
        for i in range(self.num_ice_followers + 1):  # First AV + n ICE
            offset_distance = (i + 1) * 8.0
            offset_location = base_spawn.location + carla.Location(x=offset_distance)
            follower_transform = carla.Transform(offset_location, base_spawn.rotation)

            follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.tesla.model3'
            vehicle = self.spawn_vehicle(follower_bp, follower_transform)
            if not vehicle:
                raise RuntimeError(f"Failed to spawn follower {i}.")
            print(f"[Follower {i} Spawned]")

            # Attach controller
            if self.controller_type == "FS":
                fs_controller = FollowerStopperController()
                follower = FollowerVehicle(vehicle, self.map, fs_controller, previous_vehicle,self.reference_speed)
            else:
                idm_controller = IDMController()
                follower = FollowerVehicle(vehicle, self.map, idm_controller, previous_vehicle,self.reference_speed)

            self.followers.append(follower)
            previous_vehicle = vehicle

    def run_asynchronously(self):
        self.setup_simulation()
        self.spawn_leader_and_followers()
        print(f"{self.leader.id} {self.leader.get_location()}")
        self.leader.show_debug_telemetry(enabled=True)
        # for j, follower in enumerate(self.followers):
        #     print(f"follower_ID {follower.vehicle.id} Location : {follower.vehicle.get_location()}")
        #     print(f"Leader Id {follower.leader.id} Location : {follower.leader.get_location()}")
        #     print(f'--------------------------------------------------------------------------')
        self.world.tick()

        print("Starting leader-follower simulation...")

        speed_df = self.speed_controller.df  # Convenience alias
        try:
            for i in range(1, len(speed_df)):
            # for i in range(1, 1000):
            # for i in range(1, 7000):
                # Step-specific timing and speed
                sim_time = speed_df.loc[i, 'time_rel']
                # delta_t = speed_df.loc[i, 'time_diff']
                delta_t = 0.1
                target_speed = speed_df.loc[i, 'speed_mps']
                # if target_speed == 0:
                    # print(f"Sim Time : {sim_time}")
                    # print("------------------------------")

                ## --- Leader control ---
                # direction_vector, next_wp = self.get_direction_to_next_wp(self.leader)
                # if direction_vector is None:
                #     print("Leader has no more waypoints.")
                #     break
                #
                # velocity = carla.Vector3D(
                #     direction_vector.x * target_speed
                # )
                #
                # self.leader.set_target_velocity(velocity)
                # # self.leader.set_target_angular_velocity(velocity)
                #
                # transform = self.leader.get_transform()
                # transform.rotation = next_wp.transform.rotation
                # self.leader.set_transform(transform)

                # --- Get current state and direction ---
                direction_vector, next_wp = self.get_direction_to_next_wp(self.leader)
                if direction_vector is None:
                    print("Leader has no more waypoints.")
                    break

                current_transform = self.leader.get_transform()
                current_location = current_transform.location
                current_yaw = current_transform.rotation.yaw  # In degrees

                # --- Compute target yaw ---
                target_location = next_wp.transform.location
                vec_to_wp = target_location - current_location
                angle_to_wp = np.degrees(np.arctan2(vec_to_wp.y, vec_to_wp.x))

                # --- Compute steering ---
                steering_error = angle_to_wp - current_yaw
                steering_error = (steering_error + 180) % 360 - 180  # Wrap to [-180, 180]
                steer = np.clip(steering_error / 45.0, -1.0, 1.0)  # Normalize to CARLA range

                # --- Compute throttle ---
                current_speed = self.leader.get_velocity()
                current_speed_mps = np.linalg.norm([current_speed.x, current_speed.y, current_speed.z])
                speed_error = target_speed - current_speed_mps

                throttle = np.clip(speed_error * 0.5, 0.0, 1.0)  # Proportional control
                brake = 0.0
                if speed_error < -0.5:
                    brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
                    throttle = 0.0

                # --- Apply control ---
                control = carla.VehicleControl()
                control.throttle = throttle
                control.brake = brake
                control.steer = steer
                self.leader.apply_control(control)


                self.logger.log(sim_time, 'leader', self.leader.get_location(), self.leader.get_velocity(), round(self.leader.get_acceleration().x,3))

                # --- Followers control ---
                # previous_leader_vel = velocity
                # if i > 103:
                for j, follower in enumerate(self.followers):
                    # if target_speed == 0:
                    #     print(self.leader.get_velocity())
                    #     follower.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                    label = f'follower_{j}'
                    # follower.leader.set_target_velocity(previous_leader_vel)
                    if self.controller_type == "FS":
                        command_velocity, reference_speed, rel_speed, quadratic_region = follower.update_fs()
                    else:
                        command_velocity, rel_speed = follower.update_idm(delta_t)
                        reference_speed = 0
                        quadratic_region = (0,0,0)
                    gap, leader_speed = follower.compute_gap_and_leader_speed()
                    # if gap < 5 :
                    #     print(f"Follower {j} and Leader {j-1} speed {leader_speed} : GAP : {gap} and command velocity {command_velocity}")
                    previous_leader_vel = follower.vehicle.get_velocity()
                    # print(f"Follower {follower.vehicle.id} and Leader {follower.leader.id} speed {leader_speed} : GAP : {gap} and command velocity {command_velocity}")
                    # if target_speed == 0:
                    #     print(f"Sim Time : {sim_time} leader speed {target_speed}")
                    #     print(f"{label}:{follower.vehicle.id} \nspeed {follower.vehicle.get_velocity()} \ngap {gap} \nlocation {follower.vehicle.get_location()} \n\nleader ID {follower.leader.id} \n speed {follower.leader.get_velocity()} \nlocation {follower.leader.get_location()}")
                    #     print("------------------------------")
                    # follower_acceleration = follower.get_acceleration().x
                    self.logger.log(sim_time, f'follower_{j}', follower.vehicle.get_location(),
                                    follower.vehicle.get_velocity(), follower.vehicle.get_acceleration().x,
                                    gap, command_velocity, reference_speed, rel_speed, quadratic_region)

                # --- Spectator follows last follower ---
                last_follower = self.followers[-1].vehicle
                cam_transform = last_follower.get_transform().transform(carla.Location(x=-8, z=10))
                self.spectator.set_transform(carla.Transform(cam_transform, last_follower.get_transform().rotation))

            # Sync CARLA to time step
                self.world.tick()
                # time.sleep(0.01)


        except KeyboardInterrupt:
            print("Simulation interrupted.")

        finally:
            self.cleanup()

    def run(self):
        self.setup_simulation()
        self.spawn_leader_and_followers()
        self.world.tick()


        print("Starting leader-follower simulation...")

        try:
            start_time = time.time()
            while True:
                sim_time = time.time() - start_time

                # --- Leader control ---
                target_speed = self.speed_controller.get_speed_at(sim_time)
                direction_vector, next_wp = self.get_direction_to_next_wp(self.leader)
                if direction_vector is None:
                    break
                velocity = carla.Vector3D(
                    direction_vector.x * target_speed
                )

                self.leader.set_target_velocity(velocity)
                self.leader.set_target_angular_velocity(velocity)

                transform = self.leader.get_transform()
                transform.rotation = next_wp.transform.rotation
                self.leader.set_transform(transform)

                self.logger.log(sim_time, 'leader', self.leader.get_location(), self.leader.get_velocity(),
                                round(self.leader.get_acceleration().x, 3))

                # --- Followers control ---
                previous_leader_vel = velocity
                # if i > 103:
                for j, follower in enumerate(self.followers):
                    # if target_speed == 0:
                    #     print(self.leader.get_velocity())
                    #     follower.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                    label = f'follower_{j}'
                    # follower.leader.set_target_velocity(previous_leader_vel)
                    if self.controller_type == "FS":
                        command_velocity, reference_speed = follower.update_fs()
                    else:
                        command_velocity = follower.update_idm(delta_t)
                        reference_speed = 0
                    gap, leader_speed = follower.compute_gap_and_leader_speed()
                    # if gap < 5 :
                    #     print(f"Follower {j} and Leader {j-1} speed {leader_speed} : GAP : {gap} and command velocity {command_velocity}")
                    previous_leader_vel = follower.vehicle.get_velocity()
                    # print(f"Follower {follower.vehicle.id} and Leader {follower.leader.id} speed {leader_speed} : GAP : {gap} and command velocity {command_velocity}")
                    # if target_speed == 0:
                    #     print(f"Sim Time : {sim_time} leader speed {target_speed}")
                    #     print(f"{label}:{follower.vehicle.id} \nspeed {follower.vehicle.get_velocity()} \ngap {gap} \nlocation {follower.vehicle.get_location()} \n\nleader ID {follower.leader.id} \n speed {follower.leader.get_velocity()} \nlocation {follower.leader.get_location()}")
                    #     print("------------------------------")
                    # follower_acceleration = follower.get_acceleration().x
                    self.logger.log(sim_time, f'follower_{j}', follower.vehicle.get_location(),
                                    follower.vehicle.get_velocity(), follower.vehicle.get_acceleration().x, gap,
                                    command_velocity, reference_speed)

                # --- Spectator follows last follower ---
                last_follower = self.followers[-1].vehicle
                cam_transform = last_follower.get_transform().transform(carla.Location(x=-8, z=10))
                self.spectator.set_transform(carla.Transform(cam_transform, last_follower.get_transform().rotation))

                # Sync CARLA to time step
                self.world.tick()
                time.sleep(0.01)


        except KeyboardInterrupt:
            print("Simulation interrupted.")

        finally:
            self.cleanup()

    def get_direction_to_next_wp(self, vehicle, lookahead=2.0):
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
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.world.apply_settings(settings)
        # Logging and Plots
        self.logger.save()
        self.logger.plot_trajectories()
        self.logger.plot_speeds()
        self.logger.plot_reference_velocity()
        self.logger.plot_command_velocity()
        self.logger.plot_relative_velocity()
        self.logger.plot_gap_vs_time()
        print("Vehicles destroyed. Simulation ended.")

if __name__ == '__main__':
    controller_name = "FS"
    reference_speed = 30
    custom_map_path = f'{ROOT_DIR}/routes/road_with_object.xodr'
    sim = CarlaSimulator(csv_path=f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv',custom_map_path=custom_map_path,controller_name=controller_name, num_ice_followers=4, reference_speed=reference_speed)
    sim.run_asynchronously()
