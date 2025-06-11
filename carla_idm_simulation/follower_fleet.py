import carla
import math
from follower_controller import FollowerController
from idm_controller import IDMController

class FollowerFleet:
    def __init__(self, world, blueprint_library, leader, num_followers=1, gap=10.0):
        self.world = world
        self.bp_lib = blueprint_library
        self.leader = leader
        self.num_followers = num_followers
        self.gap = gap  # meters between vehicles
        self.follower_vehicles = []
        self.controllers = []

    def spawn_followers(self):
        lead_tf = self.leader.get_transform()

        for i in range(self.num_followers):
            # Position behind leader, offset by (i+1)*gap in local X
            offset = carla.Location(x=-(i + 1) * self.gap)
            spawn_loc = lead_tf.transform(offset)
            spawn_tf = carla.Transform(spawn_loc, lead_tf.rotation)

            # Choose a random or fixed follower blueprint
            follower_bp = self.bp_lib.filter('vehicle.audi.tt')[0]
            follower = self.world.try_spawn_actor(follower_bp, spawn_tf)
            if follower is not None:
                # Apply physics damping if needed here
                idm = IDMController()
                controller = FollowerController(self.world, follower, self.leader if i == 0 else self.follower_vehicles[-1], idm)
                self.follower_vehicles.append(follower)
                self.controllers.append(controller)
            else:
                print(f"Failed to spawn follower #{i+1}")

        return self.follower_vehicles, self.controllers

    def update_all(self):
        for ctrl in self.controllers:
            ctrl.update()
