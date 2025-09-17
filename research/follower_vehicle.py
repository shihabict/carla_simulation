import math
import carla
import numpy as np
from collections import deque
from nominal_contoller import NominalController

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
        if gap>self.vehicle_length:
            gap = gap - self.vehicle_length

        return gap, lead_speed

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
        # follower_transform = self.vehicle.get_transform()
        # follower_transform.location.y = 0.0
        # follower_transform.location.z = 0.0
        # self.vehicle.set_transform(follower_transform)
        ego_speed = self.get_speed()
        gap, lead_speed = self.compute_gap_and_leader_speed()
        rel_speed = lead_speed - ego_speed

        # self.leader_speed_buffer.append(lead_speed)

        acceleration = self.idm_controller.compute_acceleration(ego_speed, lead_speed, gap)
        # acceleration = max(1.5,acceleration)
        # More aggressive acceleration when far behind
        # if gap > 20 and lead_speed > ego_speed:
        #     acceleration = min(acceleration * 1.5, self.idm_controller.a_max)

        # 3. Gentle restart logic (fixes stuck ego when leader is far and moving)
        if ego_speed < 0.1 and gap > self.idm_controller.s0 and lead_speed > 1.0:
            acceleration = max(acceleration, 2.5)  # Give a push

        target_speed = max(0.0, ego_speed + acceleration * 1)

        current_loc = self.vehicle.get_location()
        # current_wp = self.map.get_waypoint(current_loc, project_to_road=True)
        current_wp = self.map.get_waypoint(current_loc)
        current_yaw = self.vehicle.get_transform().rotation.yaw

        next_wp_list = current_wp.next(self.lookahead)
        if not next_wp_list:
            return False

        next_wp = next_wp_list[0]

        target_yaw = next_wp_list[0].transform.rotation.yaw

        yaw_error = target_yaw - current_yaw
        yaw_error = (yaw_error + 180) % 360 - 180  # Normalize to [-180,180]
        steer = np.clip(yaw_error / 45.0, 0.0, 0.0)

        target_location = next_wp.transform.location
        # next_wp.transform.location.y = 0.0
        # next_wp.transform.location.z = 0.0
        # vec_to_wp = target_location - current_loc

        # steering = self.get_steer(self.vehicle.get_transform(),next_wp.transform)

        # current_speed = self.get_speed()
        speed_error = target_speed - ego_speed
        throttle = np.clip(speed_error * 0.5, 0.0, 1.0)
        brake = 0.0
        if speed_error < -0.5:
            # brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
            brake = np.clip(-speed_error, 0.0, 1.0)
            throttle = 0.0

        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        control.steer = 0.0
        # control.steer = self.compute_lateral_control(self.leader)
        # print(f"Steering control: {control.steer}")
        if control.steer > 0.0:
            print(f"Follower Steering : {control.steer}")
        self.vehicle.apply_control(control)
        vehicle_location = self.vehicle.get_location()

        # print(
        #     f"[IDM Controller] Ego: {ego_speed:.2f} | Lead: {lead_speed:.2f} | Gap: {gap:.2f} | Ref: {reference_speed:.2f}")

        return target_speed, gap, vehicle_location, rel_speed

    def update_fs(self,reference_speed):

        # follower_transform = self.vehicle.get_transform()
        # follower_transform.location.y = 0.0
        # follower_transform.location.z = 0.0
        # self.vehicle.set_transform(follower_transform)

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
        current_yaw = self.vehicle.get_transform().rotation
        if not next_wp_list:
            return False

        next_wp = next_wp_list[0]
        target_location = next_wp.transform.location
        # next_wp.transform.location.y = 0.0
        # next_wp.transform.location.z = 0.0
        # vec_to_wp = target_location - current_loc

        # steering = self.get_steer(self.vehicle.get_transform(), next_wp.transform)

        current_speed = self.get_speed()
        speed_error = commanded_speed - current_speed
        throttle = np.clip(speed_error * 0.5, 0.0, 1.0)
        brake = 0.0
        if speed_error < -0.5:
            # brake = np.clip(-speed_error * 0.5, 0.0, 1.0)
            brake = np.clip(-speed_error, 0.0, 1.0)
            throttle = 0.0

        control = carla.VehicleControl()
        control.throttle = throttle
        control.brake = brake
        # control.steer = self.compute_lateral_control(self.leader)
        control.steer = 0.0
        if control.steer > 0.0:
            print(f"Follower Steering : {control.steer}")
        self.vehicle.apply_control(control)
        vehicle_location = self.vehicle.get_location()

        # print(
        #     f"[FS Controller] Ego: {ego_speed:.2f} | Lead: {lead_speed:.2f} | Gap: {gap:.2f} | Ref: {reference_speed:.2f}")

        return commanded_speed,gap,vehicle_location, rel_speed
