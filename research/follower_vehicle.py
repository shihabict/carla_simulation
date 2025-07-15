import carla
import numpy as np

from nominal_contoller import NominalController


# from idm_controller import IDMController

class FollowerVehicle:
    def __init__(self, vehicle_actor: carla.Vehicle, map_ref: carla.Map,
                 controller, leader_vehicle: carla.Vehicle, reference_speed,waypoint_lookahead=2.0):
        self.vehicle = vehicle_actor
        self.map = map_ref
        self.controller = controller
        self.leader = leader_vehicle
        self.lookahead = waypoint_lookahead
        self.vehicle_length = 4.6
        self.reference_speed = reference_speed
        self.nominal_controller = NominalController(dt=0.01, reference_speed=self.reference_speed)

    def get_speed(self):
        v = self.vehicle.get_velocity()
        return np.linalg.norm([v.x])

    def compute_gap_and_leader_speed(self):
        ego_loc = self.vehicle.get_location()
        leader_loc = self.leader.get_location()
        gap = ego_loc.distance(leader_loc)
        # gap = round(gap,3)

        lead_velocity = self.leader.get_velocity()
        lead_speed = np.linalg.norm([lead_velocity.x],ord=2)
        # lead_speed = lead_velocity
        # substruct vehicle length from the distance
        gap = gap - self.vehicle_length

        return gap, lead_speed

    def update_idm(self, delta_t):
        # 1. Get current speed, leader gap and speed
        ego_speed = self.get_speed()
        gap, lead_speed = self.compute_gap_and_leader_speed()

        rel_speed = lead_speed - ego_speed

        # 2. IDM acceleration
        acceleration = self.controller.compute_acceleration(ego_speed, lead_speed, gap)

        # Deadlock fix: gently push if leader is moving and gap is large enough
        # if ego_speed < 0.01 and lead_speed > 0.5 and gap > self.controller.s0 + 2.0:
        #     acceleration = 0.5  # gentle push to start moving again
        #     print(f"[RESTART] Follower recovering from stop: gap={gap:.2f}, lead_speed={lead_speed:.2f}")

        # 3. Integrate speed
        target_speed = max(0.0, ego_speed + acceleration * delta_t)
        # target_speed = ego_speed + acceleration * delta_t
        # if target_speed == 0:
        #     print(f"EGO SPEED : {ego_speed}")
        #     print(f"Acceleration : {acceleration}")
        #     print(f"Delta t : {delta_t}")
        # target_speed = min(0.0, ego_speed + acceleration * delta_t)

        # 4. Get forward direction from waypoint
        current_loc = self.vehicle.get_location()
        current_wp = self.map.get_waypoint(current_loc, project_to_road=True)
        next_wp_list = current_wp.next(self.lookahead)

        if not next_wp_list:
            return False  # No more waypoints

        next_wp = next_wp_list[0]
        direction = (next_wp.transform.location - current_loc).make_unit_vector()

        # 5. Apply velocity
        velocity = carla.Vector3D(direction.x * target_speed,
                                  direction.y * target_speed,
                                  direction.z * target_speed)
        self.vehicle.set_target_velocity(velocity)
        self.leader.set_target_angular_velocity(velocity)

        # 6. Apply yaw alignment
        transform = self.vehicle.get_transform()
        transform.rotation = next_wp.transform.rotation
        self.vehicle.set_transform(transform)

        return np.linalg.norm([velocity.x]),rel_speed

    def update_fs(self):
        # 1. Get current speed and gap
        ego_speed = self.get_speed()
        gap, lead_speed = self.compute_gap_and_leader_speed()

        rel_speed = lead_speed - ego_speed  # dv

        # compute reference velocity
        reference_speed = self.nominal_controller.get_reference_speed(ego_speed)

        # 2. FollowerStopper: compute commanded velocity
        # reference_speed = 15.0  # fixed for now
        # print(f"")
        commanded_speed, _ = self.controller.compute_velocity_command(
            r=reference_speed,
            dx=gap,
            dv=rel_speed,
            v_AV=ego_speed
        )
        # commanded_speed = round(commanded_speed,3)

        # 3. Get direction from waypoint
        current_loc = self.vehicle.get_location()
        current_wp = self.map.get_waypoint(current_loc, project_to_road=True)
        next_wp_list = current_wp.next(self.lookahead)

        if not next_wp_list:
            return False  # No more waypoints

        next_wp = next_wp_list[0]
        direction = (next_wp.transform.location - current_loc).make_unit_vector()

        # 4. Apply target velocity
        velocity = carla.Vector3D(direction.x * commanded_speed)
        self.vehicle.set_target_velocity(velocity)
        self.leader.set_target_angular_velocity(velocity)

        # 5. Apply yaw alignment
        transform = self.vehicle.get_transform()
        transform.rotation = next_wp.transform.rotation
        self.vehicle.set_transform(transform)

        return commanded_speed, reference_speed, rel_speed

