import carla
import random
import time

from idm_controller import IDMController
from follower_vehicle import FollowerVehicle
from research.real_data_simulation import SpeedProfileController
from sim_logger import SimulationLogger



class CarlaSimulator:
    def __init__(self, csv_path, num_ice_followers=3):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()

        self.csv_path = csv_path
        self.num_ice_followers = num_ice_followers

        self.leader = None
        self.followers = []
        self.spectator = self.world.get_spectator()
        self.logger = SimulationLogger()

    def setup_simulation(self):
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        self.delta_t = settings.fixed_delta_seconds
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
        self.leader = self.spawn_vehicle(leader_bp, base_spawn)
        if not self.leader:
            raise RuntimeError("Failed to spawn leader vehicle.")
        print("[Leader Spawned]")

        # Spawn Followers (10 meters apart behind the leader)
        previous_vehicle = self.leader
        for i in range(self.num_ice_followers + 1):  # First AV + n ICE
            offset_distance = -(i + 1) * 10.0
            offset_location = base_spawn.location + carla.Location(x=offset_distance)
            follower_transform = carla.Transform(offset_location, base_spawn.rotation)

            follower_bp = 'vehicle.audi.tt' if i == 0 else 'vehicle.tesla.model3'
            vehicle = self.spawn_vehicle(follower_bp, follower_transform)
            if not vehicle:
                raise RuntimeError(f"Failed to spawn follower {i}.")
            print(f"[Follower {i} Spawned]")

            # Attach IDM controller
            idm = IDMController()
            follower = FollowerVehicle(vehicle, self.map, idm, previous_vehicle)
            self.followers.append(follower)
            previous_vehicle = vehicle

    def run(self):
        self.setup_simulation()
        self.spawn_leader_and_followers()
        self.world.tick()

        speed_controller = SpeedProfileController(self.csv_path)
        print("Starting leader-follower simulation...")

        try:
            start_time = time.time()
            while True:
                sim_time = time.time() - start_time

                # --- Leader control ---
                target_speed = speed_controller.get_speed_at(sim_time)
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

                # --- Followers control ---
                for follower in self.followers:
                    success = follower.update(self.delta_t)
                    if not success:
                        print("Follower lost waypoint.")
                        break

                # Spectator follow leader
                cam_transform = self.leader.get_transform().transform(carla.Location(x=-10, z=3))
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
        print("Vehicles destroyed. Simulation ended.")
