import math
import carla
import time

class FollowerController:

    def __init__(self, world, follower, leader, idm_controller, fs_controller, switch_time=50):
        self.world = world
        self.map = world.get_map()
        self.follower = follower
        self.leader = leader
        self.idm = idm_controller
        self.fs = fs_controller
        self.switch_time = switch_time
        self.start_time = time.time()
        self.last_steer = 0.0

    def get_speed(self, vel):
        return round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2))
        # return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    def update(self):

        elapsed_time = time.time() - self.start_time

        leader_tf = self.leader.get_transform()
        follower_tf = self.follower.get_transform()

        leader_vel = self.leader.get_velocity()
        follower_vel = self.follower.get_velocity()

        v_lead = self.get_speed(leader_vel)
        v_follower = self.get_speed(follower_vel)
        delta_x = leader_tf.location.distance(follower_tf.location)

        # === Choose controller based on time ===
        if int(elapsed_time // self.switch_time) % 2 == 0:
            throttle= self.idm.step(delta_x,v_follower, v_lead)
        else:
            v_cmd = self.fs.compute_command_velocity(v_follower, v_lead, delta_x)
            speed_error = v_cmd - v_follower
            throttle = max(0.0, min(speed_error * 0.2, 1.0))
            # throttle = v_cmd
            brake = max(0.0, min(-speed_error * 0.3, 1.0))

        # === Lateral Control ===
        follower_wp = self.map.get_waypoint(follower_tf.location, project_to_road=True)
        lookahead_wp = follower_wp.next(6.0)[0] if follower_wp.next(6.0) else follower_wp

        target_loc = lookahead_wp.transform.location
        v_target = target_loc - follower_tf.location
        v_forward = follower_tf.get_forward_vector()

        dot = v_forward.x * v_target.x + v_forward.y * v_target.y
        cross = v_forward.x * v_target.y - v_forward.y * v_target.x
        angle = math.atan2(cross, dot)

        # Filtered (smooth) steering
        raw_steer = max(-1.0, min(1.0, angle * 2.0))
        alpha = 0.3
        steer = (1 - alpha) * self.last_steer + alpha * raw_steer
        self.last_steer = steer

        control = carla.VehicleControl(throttle=throttle,steer=0)
        self.follower.apply_control(control)
