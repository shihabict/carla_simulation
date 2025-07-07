import math
import carla
import time
import pandas as pd

class LeaderController:
    def __init__(self, vehicle, waypoints, speed_csv_path):
        self.vehicle = vehicle
        self.waypoints = waypoints
        self.index = 0
        self.start_time = time.time()
        # Load CSV speed profile
        df = pd.read_csv(speed_csv_path)
        df = df[df['Message'] > 0].reset_index(drop=True)
        self.time_series = df['Time'].values - df['Time'].values[0]  # Normalize to start at 0
        self.speed_series = df['Message'].values / 3.6  # Convert km/h to m/s

    def get_speed(self, vel):
        return round(math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2))
        # return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    def get_speed_from_csv(self, elapsed_time):
        # Interpolate speed from CSV
        if elapsed_time <= self.time_series[0]:
            return self.speed_series[0]
        elif elapsed_time >= self.time_series[-1]:
            return self.speed_series[-1]
        else:
            for i in range(len(self.time_series) - 1):
                if self.time_series[i] <= elapsed_time < self.time_series[i + 1]:
                    t1, t2 = self.time_series[i], self.time_series[i + 1]
                    v1, v2 = self.speed_series[i], self.speed_series[i + 1]
                    ratio = (elapsed_time - t1) / (t2 - t1)
                    return v1 + ratio * (v2 - v1)
            return self.speed_series[-1]

    def get_target_speed(self, elapsed_time):
        cycle_time = elapsed_time % 25  # Repeat every 25 seconds

        if cycle_time < 10:
            return 15.0  # High speed for first 10 seconds
        else:
            return 7.0  # Low speed for next 5 seconds

    def run_step(self):
        if self.index >= len(self.waypoints):
            return

        current_time = time.time()
        elapsed_time = current_time - self.start_time
        # target_speed = self.get_target_speed(elapsed_time)
        target_speed = self.get_speed_from_csv(elapsed_time)

        # Get transform
        vehicle_tf = self.vehicle.get_transform()
        vehicle_loc = vehicle_tf.location
        vehicle_vec = vehicle_tf.get_forward_vector()

        # Get target waypoint
        target_wp = self.waypoints[self.index]
        target_loc = target_wp.transform.location

        # Compute steering angle
        v_target = target_loc - vehicle_loc
        dot = vehicle_vec.x * v_target.x + vehicle_vec.y * v_target.y
        cross = vehicle_vec.x * v_target.y - vehicle_vec.y * v_target.x
        angle = math.atan2(cross, dot)
        # print(f"Steering Angle: {angle}")
        # steer = max(min(angle * 2.0, 1.0), -1.0)

        # --- Speed Control ---
        current_speed = self.get_speed(self.vehicle.get_velocity())
        speed_error = target_speed - current_speed

        # Proportional speed control (simple)
        throttle = max(0.0, min(speed_error * 0.2, 1.0))  # gain = 0.2
        brake = max(0.0, min(-speed_error * 0.3, 1.0))    # gain = 0.3

        # control = carla.VehicleControl(throttle=throttle,steer=0)
        control = carla.VehicleControl(throttle=throttle,steer=0)
        self.vehicle.apply_control(control)
