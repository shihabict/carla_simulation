import math
import carla

# In follower_controller.py (create if missing)
class FollowerController:
    def __init__(self, world, follower, leader, idm_controller):
        self.world = world
        self.follower = follower
        self.leader = leader
        self.idm = idm_controller
        self.map = world.get_map()

    def update(self):
        leader_tf = self.leader.get_transform()
        follower_tf = self.follower.get_transform()

        # Longitudinal control (your existing IDM)
        dx = leader_tf.location.distance(follower_tf.location)
        leader_vel = self.leader.get_velocity()
        follower_vel = self.follower.get_velocity()
        dv = math.sqrt(leader_vel.x ** 2 + leader_vel.y ** 2) - \
             math.sqrt(follower_vel.x ** 2 + follower_vel.y ** 2)
        throttle, brake = self.idm.step(dx, dv)

        # === NEW: Lateral control ===
        # Get waypoints 5m ahead of both vehicles
        leader_wp = self.map.get_waypoint(leader_tf.location, project_to_road=True)
        follower_wp = self.map.get_waypoint(follower_tf.location, project_to_road=True)

        # Calculate target yaw (simplified)
        target_yaw = math.atan2(
            leader_wp.transform.location.y - follower_wp.transform.location.y,
            leader_wp.transform.location.x - follower_wp.transform.location.x
        )

        # Steering PID (simple version)
        yaw_error = (target_yaw - math.radians(follower_tf.rotation.yaw)) % (2 * math.pi)
        if yaw_error > math.pi:
            yaw_error -= 2 * math.pi

        steer = max(-1.0, min(1.0, yaw_error * 0.5))  # Proportional control

        # Apply combined control
        control = carla.VehicleControl(
            throttle=throttle,
            brake=brake,
            steer=steer
        )
        self.follower.apply_control(control)