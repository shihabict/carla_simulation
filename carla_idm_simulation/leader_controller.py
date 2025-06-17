import math
import carla
import time

class LeaderController:
    def __init__(self, vehicle, waypoints):
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.index = 0
        self.start_time = time.time()

    def get_speed(self, vel):
        return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    def get_target_speed(self, elapsed_time):
        cycle_time = elapsed_time % 25  # Repeat every 15 seconds

        if cycle_time < 10:
            return 10.0  # High speed for first 10 seconds
        else:
            return 7.0  # Low speed for next 5 seconds

    def run_step(self):
        if self.index >= len(self.waypoints):
            return

        current_time = time.time()
        elapsed_time = current_time - self.start_time
        target_speed = self.get_target_speed(elapsed_time)

        # Get transform
        vehicle_tf = self.vehicle.get_transform()
        vehicle_loc = vehicle_tf.location
        vehicle_vec = vehicle_tf.get_forward_vector()

        # Get target waypoint
        target_wp = self.waypoints[self.index]
        target_loc = target_wp.transform.location

        # print(f"Distance is {vehicle_loc.distance(target_loc)}")

        if vehicle_loc.distance(target_loc) < 2.0:
            # print(f"Distance is {vehicle_loc.distance(target_loc)}")
            self.index += 1
            return

        # Compute steering angle
        v_target = target_loc - vehicle_loc
        dot = vehicle_vec.x * v_target.x + vehicle_vec.y * v_target.y
        cross = vehicle_vec.x * v_target.y - vehicle_vec.y * v_target.x
        angle = math.atan2(cross, dot)
        # print(f"Steering Angle: {angle}")
        steer = max(min(angle * 2.0, 1.0), -1.0)

        # --- Speed Control ---
        current_speed = self.get_speed(self.vehicle.get_velocity())
        speed_error = target_speed - current_speed

        # Proportional speed control (simple)
        throttle = max(0.0, min(speed_error * 0.2, 1.0))  # gain = 0.2
        brake = max(0.0, min(-speed_error * 0.3, 1.0))    # gain = 0.3

        control = carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)
        self.vehicle.apply_control(control)
