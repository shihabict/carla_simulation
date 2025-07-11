import carla
import random
import time

from idm_controller import IDMController
from follower_vehicle import FollowerVehicle
from fs_controller import FollowerStopperController
from research.speed_profiler import SpeedProfileController
# from research.real_data_simulation import SpeedProfileController
from research.utils import create_world_from_custom_map
from settings import ROOT_DIR
from sim_logger import SimulationLogger
from utils import load_xodr


class CarlaSimulator:
    def __init__(self, csv_path, custom_map_path, num_ice_followers=3):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.load_custom_map(self.client, custom_map_path)
        # self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()

        self.csv_path = csv_path
        self.speed_controller = SpeedProfileController(self.csv_path)
        self.num_ice_followers = num_ice_followers

        self.leader = None
        self.followers = []
        self.spectator = self.world.get_spectator()
        self.logger = SimulationLogger()

    def load_custom_map(self,client, custom_map_path):
        xodr_content = load_xodr(custom_map_path)
        world = create_world_from_custom_map(client, xodr_content)
        return world

    def setup_simulation(self):
        settings = self.world.get_settings()
        # settings.synchronous_mode = True
        settings.synchronous_mode = False
        # settings.fixed_delta_seconds = 0.05
        settings.fixed_delta_seconds = None
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

        # Spawn Followers (10 meters apart behind the leader)
        previous_vehicle = self.leader
        for i in range(self.num_ice_followers + 1):  # First AV + n ICE
            offset_distance = (i + 1) * 20.0
            offset_location = base_spawn.location + carla.Location(x=offset_distance)
            follower_transform = carla.Transform(offset_location, base_spawn.rotation)

            follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.tesla.model3'
            vehicle = self.spawn_vehicle(follower_bp, follower_transform)
            if not vehicle:
                raise RuntimeError(f"Failed to spawn follower {i}.")
            print(f"[Follower {i} Spawned]")

            # Attach IDM controller
            # idm = IDMController()
            # follower = FollowerVehicle(vehicle, self.map, idm, previous_vehicle)

            # Attach FS
            fs_controller = FollowerStopperController()
            follower = FollowerVehicle(vehicle, self.map, fs_controller, previous_vehicle)

            self.followers.append(follower)
            previous_vehicle = vehicle

    def run_asynchronously(self):
        self.setup_simulation()
        self.spawn_leader_and_followers()
        self.world.tick()

        print("Starting leader-follower simulation...")

        speed_df = self.speed_controller.df  # Convenience alias
        try:
            # for i in range(1, len(speed_df)):
            for i in range(100, 7000):
                # Step-specific timing and speed
                sim_time = speed_df.loc[i, 'time_rel']
                delta_t = speed_df.loc[i, 'time_diff']
                target_speed = speed_df.loc[i, 'speed_mps']
                # if target_speed == 0:
                    # print(f"Sim Time : {sim_time}")
                    # print("------------------------------")

                # --- Leader control ---
                direction_vector, next_wp = self.get_direction_to_next_wp(self.leader)
                if direction_vector is None:
                    print("Leader has no more waypoints.")
                    break

                velocity = carla.Vector3D(
                    direction_vector.x * target_speed,
                    direction_vector.y * target_speed,
                    direction_vector.z * target_speed
                )
                # if target_speed == 0:
                #     print(velocity)
                #     previous_velocity = self.leader.get_velocity()
                #     print(previous_velocity)
                #     print('*******************************************')
                #     self.leader.set_target_velocity(velocity)
                #     updated_velocity = self.leader.get_velocity()
                #     print(updated_velocity)

                self.leader.set_target_velocity(velocity)
                self.leader.set_target_angular_velocity(velocity)


                transform = self.leader.get_transform()
                transform.rotation = next_wp.transform.rotation
                self.leader.set_transform(transform)

                self.logger.log(sim_time, 'leader', self.leader.get_location(), self.leader.get_velocity())

                # --- Followers control ---
                previous_leader_vel = velocity
                for j, follower in enumerate(self.followers):
                    if target_speed == 0:
                        print(self.leader.get_velocity())
                    #     follower.vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                    label = f'follower_{j}'
                    # follower.leader.set_target_velocity(previous_leader_vel)
                    follower.update_fs()
                    gap, leader_speed = follower.compute_gap_and_leader_speed()
                    previous_leader_vel = follower.vehicle.get_velocity()
                    # if target_speed == 0:
                    #     print(f"Sim Time : {sim_time} leader speed {target_speed}")
                    #     print(f"{label}:{follower.vehicle.id} \nspeed {follower.vehicle.get_velocity()} \ngap {gap} \nlocation {follower.vehicle.get_location()} \n\nleader ID {follower.leader.id} \n speed {follower.leader.get_velocity()} \nlocation {follower.leader.get_location()}")
                    #     print("------------------------------")
                    self.logger.log(sim_time, f'follower_{j}', follower.vehicle.get_location(),
                                    follower.vehicle.get_velocity(), gap)

                    # self.logger.log(sim_time, label, follower.vehicle.get_location(), follower.vehicle.get_velocity())

                    # Optional: draw debug label
                    # self.world.debug.draw_string(
                    #     follower.vehicle.get_location() + carla.Location(z=2.5),
                    #     label,
                    #     draw_shadow=False,
                    #     color=carla.Color(255, 255, 0),
                    #     life_time=0.1,
                    #     persistent_lines=False
                    # )

                # --- Spectator follows last follower ---
                last_follower = self.followers[-1].vehicle
                cam_transform = last_follower.get_transform().transform(carla.Location(x=-8, z=10))
                self.spectator.set_transform(carla.Transform(cam_transform, last_follower.get_transform().rotation))

                # Sync CARLA to time step
                self.world.tick()
                time.sleep(delta_t)


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
                velocity = carla.Vector3D(direction_vector.x * target_speed,
                                          direction_vector.y * target_speed,
                                          direction_vector.z * target_speed)
                self.leader.set_target_velocity(velocity)
                transform = self.leader.get_transform()
                transform.rotation = next_wp.transform.rotation
                self.leader.set_transform(transform)
                leader_loc = self.leader.get_location()
                # self.world.debug.draw_string(
                #     leader_loc + carla.Location(z=2.5),
                #     "Leader",
                #     draw_shadow=False,
                #     color=carla.Color(0, 255, 255),  # Cyan
                #     life_time=0.1
                # )

                self.logger.log(sim_time, 'leader', self.leader.get_location(), self.leader.get_velocity())

                # --- Followers control ---
                # for follower in self.followers:
                #     success = follower.update(self.delta_t)
                #     if not success:
                #         print("Follower lost waypoint.")
                #         break

                for i, follower in enumerate(self.followers):
                    label = f'follower_{i}'
                    # loc = follower.vehicle.get_location()
                    # self.world.debug.draw_string(
                    #     loc + carla.Location(z=2.5),  # Raise text above the vehicle
                    #     label,
                    #     draw_shadow=False,
                    #     color=carla.Color(255, 255, 0),  # Yellow
                    #     life_time=0.1
                    # )
                    success = follower.update_fs(self.delta_t)
                    self.logger.log(sim_time, label, follower.vehicle.get_location(), follower.vehicle.get_velocity())


                    if not success:
                        print(f"[WARNING] {label} lost waypoint. Exiting...")
                        break

                # Spectator follow leader
                spectator_vehicle = self.followers[-1].vehicle
                cam_transform = spectator_vehicle.get_transform().transform(carla.Location(x=10, z=10))
                self.spectator.set_transform(carla.Transform(cam_transform, self.leader.get_transform().rotation))

                self.world.tick()


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
        self.logger.plot_gap_vs_time()
        print("Vehicles destroyed. Simulation ended.")

if __name__ == '__main__':
    custom_map_path = f'{ROOT_DIR}/routes/road_with_object.xodr'
    sim = CarlaSimulator(f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv',custom_map_path)
    sim.run_asynchronously()
