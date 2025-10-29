import math
import carla
import numpy as np
from collections import deque
from nominal_contoller import NominalController
from research.pid_controller import LaneCenteringController
from research.utils import compute_steer_towards


class FollowerVehicle:
    def __init__(self, vehicle_actor: carla.Vehicle, map_ref: carla.Map,
                 leader_vehicle: carla.Vehicle, reference_speed, idm_controller, fs_controller, waypoint_lookahead=2.0):
        self.vehicle = vehicle_actor
        self.map = map_ref
        self.leader = leader_vehicle
        self.lookahead = waypoint_lookahead
        self.vehicle_length = 4.79
        self.reference_speed = reference_speed

        # Controllers
        self.idm_controller = idm_controller
        self.fs_controller = fs_controller
        # self.idm_controller = IDMController()
        # self.fs_controller = FollowerStopperController()
        self.nominal_controller = NominalController(dt=0.01, reference_speed=self.reference_speed)

        # New: Leader speed buffer (rolling window of last 200 values)
        self.leader_speed_buffer = deque(maxlen=200)

        self.previous_y_error = 0.0
        self.previous_steer = 0.0


    def get_speed(self):
        v = self.vehicle.get_velocity()
        return np.linalg.norm([v.x, v.y])

    def compute_gap_and_leader_speed(self):
        ego_loc = self.vehicle.get_location()
        leader_loc = self.leader.get_location()
        gap = ego_loc.distance(leader_loc)
        # gap = leader_loc.distance(ego_loc)

        lead_velocity = self.leader.get_velocity()
        lead_speed = np.linalg.norm([lead_velocity.x,lead_velocity.y])
        # if gap>self.vehicle_length:
        gap = gap - self.vehicle_length

        return gap, lead_speed

    def get_lookahead_waypoint(self, world_map, vehicle, lookahead_dist=8.0):
        """
        world_map: carla.Map
        vehicle: carla.Actor (Vehicle)
        lookahead_dist: meters ahead we want to aim for
        returns: (target_waypoint, current_waypoint)
        """
        transform = vehicle.get_transform()
        loc = transform.location

        # snap to nearest driving lane center
        current_wp = world_map.get_waypoint(
            loc,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        dist = 0.0
        wp_ahead = current_wp
        step = 1.0  # meters to march each call
        while dist < lookahead_dist:
            nxt = wp_ahead.next(step)
            if not nxt:
                break
            wp_ahead = nxt[0]
            dist += step

        return wp_ahead, current_wp

    import math

    def compute_steer_towards(self, world_map, vehicle, lookahead_dist=8.0, max_abs_steer=0.6):
        """
        Returns a steering command in [-1, 1] to aim the car toward the lane center ahead.
        """
        target_wp, _ = self.get_lookahead_waypoint(world_map, vehicle, lookahead_dist)

        veh_tf = vehicle.get_transform()
        veh_loc = veh_tf.location
        veh_yaw_deg = veh_tf.rotation.yaw
        veh_yaw = math.radians(veh_yaw_deg)

        # heading unit vector of the vehicle in world frame
        heading_vec = carla.Vector3D(math.cos(veh_yaw), math.sin(veh_yaw), 0.0)

        # vector from vehicle to target point in world frame
        to_target = carla.Vector3D(
            target_wp.transform.location.x - veh_loc.x,
            target_wp.transform.location.y - veh_loc.y,
            0.0
        )

        # signed angle between heading_vec and to_target (2D)
        dot = heading_vec.x * to_target.x + heading_vec.y * to_target.y
        det = heading_vec.x * to_target.y - heading_vec.y * to_target.x
        angle_err = math.atan2(det, dot)  # + = need to steer left, - = right

        # map angle to steering command
        steer_cmd = angle_err / 0.6  # scale ~proportional
        steer_cmd = max(-max_abs_steer, min(max_abs_steer, steer_cmd))

        # normalize to [-1, 1] for CARLA VehicleControl.steer
        steer_cmd = steer_cmd / max_abs_steer
        return float(steer_cmd)

    def compute_lateral_control(self, vehicle, target_y=0.0):
        current_y = vehicle.get_location().y
        y_error = target_y - current_y
        # Simple proportional control
        steer = np.clip(y_error * 0.1, -0.2, 0.2)  # Limit steering
        return steer

    def get_steer(self, vehicle_transform, waypoint_transform):
        """
        Calculate steering angle needed to follow the center line.
        """
        vehicle_location = vehicle_transform.location
        vehicle_yaw = math.radians(vehicle_transform.rotation.yaw)
        waypoint_location = waypoint_transform.location

        # Compute vector from vehicle to waypoint
        dx = waypoint_location.x - vehicle_location.x
        dy = waypoint_location.y - vehicle_location.y
        target_yaw = math.atan2(dy, dx)
        angle_diff = target_yaw - vehicle_yaw

        # Normalize angle to [-pi, pi]
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi

        # CARLA steering is in [-1, 1] range, so scale if needed
        steer = max(-0.0, min(0.0, angle_diff))
        return steer

    def update_idm(self):
        ego_speed = self.get_speed()
        gap, lead_speed = self.compute_gap_and_leader_speed()
        rel_speed = lead_speed - ego_speed

        # self.leader_speed_buffer.append(lead_speed)

        acceleration = self.idm_controller.compute_acceleration(ego_speed, lead_speed, gap)


        # 3. Gentle restart logic (fixes stuck ego when leader is far and moving)
        if ego_speed < 0.1 and gap > self.idm_controller.s0 and lead_speed > 1.0:
            acceleration = max(acceleration, 2.5)  # Give a push

        target_speed = max(0.0, ego_speed + acceleration * 1)



        # current_speed = self.get_speed()
        speed_error = target_speed - ego_speed
        throttle = np.clip(speed_error * 0.5, 0.0, 1.0)
        brake = 0.0
        if speed_error < -0.5:
            # brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
            brake = np.clip(-speed_error, 0.0, 1.0)
            throttle = 0.0

        steer_cmd = compute_steer_towards(self.map, self.vehicle,
                                          lookahead_dist=8.0,
                                          max_abs_steer=0.6)

        control = carla.VehicleControl(
            throttle=float(throttle),
            brake=float(brake),
            steer=float(steer_cmd)
        )
        vehicle_location = self.vehicle.get_location()

        self.vehicle.apply_control(control)

        # control = carla.VehicleControl()
        # control.throttle = throttle
        # control.brake = brake
        # if 0<ego_speed<10:
        #     steer = 0.0
        #     self.previous_steer = 0.0
        #     self.previous_y_error = 0.0
        # control.steer = 0.0
        # control.steer = self.compute_lateral_control(self.leader)
        # print(f"Steering control: {control.steer}")
        if control.steer > 0.0:
            print(f"Follower Steering : {control.steer}")
        # self.vehicle.apply_control(control)


        # print(
        #     f"[IDM Controller] Ego: {ego_speed:.2f} | Lead: {lead_speed:.2f} | Gap: {gap:.2f} | Ref: {reference_speed:.2f}")

        return target_speed, gap, vehicle_location, rel_speed

    def update_fs(self,reference_speed):

        ego_speed = self.get_speed()
        gap, lead_speed = self.compute_gap_and_leader_speed()
        rel_speed = lead_speed - ego_speed

        commanded_speed, quadratic_regions = self.fs_controller.compute_velocity_command(
            r=reference_speed,
            dx=gap,
            dv=rel_speed,
            v_AV=ego_speed
        )

        current_loc = self.vehicle.get_location()
        # current_loc.y = 0.00
        # current_loc.z = 0.00
        current_wp = self.map.get_waypoint(current_loc, project_to_road=True)
        next_wp_list = current_wp.next(self.lookahead)
        # current_yaw = self.vehicle.get_transform().rotation
        if not next_wp_list:
            return False

        current_speed = self.get_speed()
        speed_error = commanded_speed - current_speed
        throttle = np.clip(speed_error * 0.5, 0.0, 1.0)
        brake = 0.0
        if speed_error < -0.5:
            # brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
            brake = np.clip(-speed_error, 0.0, 1.0)
            throttle = 0.0

        # # Lateral control for lane keeping
        # current_y = self.vehicle.get_location().y
        # target_y = 1.75  # Lane center
        # y_error = target_y - current_y
        #
        # # Proportional steering control
        # if y_error != 0:
        #     steer = np.clip(y_error * 0.2, -0.1, 0.1)  # Gentle steering
        # else:
        #     steer = 0.0

        steer_cmd = compute_steer_towards(self.map, self.vehicle,
                                          lookahead_dist=8.0,
                                          max_abs_steer=0.6)

        control = carla.VehicleControl(
            throttle=float(throttle),
            brake=float(brake),
            steer=float(steer_cmd)
        )
        vehicle_location = self.vehicle.get_location()

        self.vehicle.apply_control(control)
        # if control.steer > 0.0:
        #     print(f"Follower Steering : {control.steer}")
        # self.vehicle.apply_control(control)
        # vehicle_location = self.vehicle.get_location()

        # print(
        #     f"[FS Controller] Ego: {ego_speed:.2f} | Lead: {lead_speed:.2f} | Gap: {gap:.2f} | Ref: {reference_speed:.2f}")

        return commanded_speed,gap,vehicle_location, rel_speed
