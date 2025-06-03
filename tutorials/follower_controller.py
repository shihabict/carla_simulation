# import math
# import carla
#
# class FollowerController:
#     def __init__(self, world, follower, leader, idm):
#         self.world = world
#         self.map = world.get_map()
#         self.follower = follower
#         self.leader = leader
#         self.idm = idm
#
#     def get_speed(self, velocity):
#         return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
#
#     def run_step(self):
#         # --- Get states ---
#         follower_tf = self.follower.get_transform()
#         leader_tf = self.leader.get_transform()
#         follower_vel = self.follower.get_velocity()
#         leader_vel = self.leader.get_velocity()
#
#         # --- Waypoint Following: Compute Steering ---
#         waypoint = self.map.get_waypoint(follower_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
#         target_loc = waypoint.transform.location
#         v_target = target_loc - follower_tf.location
#         v_forward = follower_tf.get_forward_vector()
#
#         dot = v_forward.x * v_target.x + v_forward.y * v_target.y
#         cross = v_forward.x * v_target.y - v_forward.y * v_target.x
#         angle = math.atan2(cross, dot)
#
#         steer = max(min(angle * 2.0, 1.0), -1.0)  # Proportional steering
#
#         # --- IDM for Throttle/Brake ---
#         dx = leader_tf.location.distance(follower_tf.location)
#         dv = self.get_speed(leader_vel) - self.get_speed(follower_vel)
#         throttle, brake = self.idm.step(dx, dv)
#
#         # --- Apply Control ---
#         control = carla.VehicleControl()
#         control.throttle = throttle
#         control.brake = brake
#         control.steer = steer
#         self.follower.apply_control(control)


import math
import carla


class FollowerController:
    def __init__(self, world, follower, leader, idm):
        self.world = world
        self.map = world.get_map()
        self.follower = follower
        self.leader = leader
        self.idm = idm

    def get_speed(self, velocity):
        return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

    def run_step(self):
        # --- Get states ---
        follower_tf = self.follower.get_transform()
        leader_tf = self.leader.get_transform()
        follower_vel = self.follower.get_velocity()
        leader_vel = self.leader.get_velocity()

        # --- Waypoint Following: Compute Steering ---
        # Get current lane waypoint and look 5 meters ahead
        current_wp = self.map.get_waypoint(follower_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        next_wps = current_wp.next(5.0)
        if not next_wps:
            print("No waypoint ahead. Skipping control step.")
            return

        lookahead_wp = next_wps[0]
        target_loc = lookahead_wp.transform.location

        v_target = target_loc - follower_tf.location
        v_forward = follower_tf.get_forward_vector()

        # Compute steering angle using atan2 of cross and dot products
        dot = v_forward.x * v_target.x + v_forward.y * v_target.y
        cross = v_forward.x * v_target.y - v_forward.y * v_target.x
        angle = math.atan2(cross, dot)

        # Proportional steering control
        steer = max(min(angle * 2.0, 1.0), -1.0)

        # --- IDM for Throttle/Brake ---
        dx = leader_tf.location.distance(follower_tf.location)
        dv = self.get_speed(leader_vel) - self.get_speed(follower_vel)
        throttle, brake = self.idm.step(dx, dv)

        # --- Apply Control ---
        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        control.steer = steer
        self.follower.apply_control(control)
