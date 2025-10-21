import carla
import math
import numpy as np


class LaneCenteringController:
    """
    PID-based lane centering controller for CARLA vehicles
    Keeps the vehicle centered in its lane using lateral error feedback
    """

    def __init__(self, vehicle, world, Kp=0.5, Ki=0.0, Kd=0.3, target_speed=30.0):
        """
        Initialize the lane centering controller

        Args:
            vehicle: CARLA vehicle actor
            world: CARLA world object
            Kp: Proportional gain for lateral control
            Ki: Integral gain for lateral control
            Kd: Derivative gain for lateral control
            target_speed: Desired speed in km/h
        """
        self.vehicle = vehicle
        self.world = world
        self.map = world.get_map()

        # PID parameters for lateral control
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # PID state variables
        self.lateral_error_integral = 0.0
        self.previous_lateral_error = 0.0

        # Speed control
        self.target_speed = target_speed / 3.6  # Convert to m/s
        self.speed_Kp = 0.5

        # Lookahead distance for target waypoint
        self.lookahead_distance = 10.0

    def get_lateral_error(self):
        """
        Calculate lateral deviation from lane center
        Returns: (lateral_error, heading_error)
        """
        # Get vehicle's current transform
        vehicle_transform = self.vehicle.get_transform()
        vehicle_location = vehicle_transform.location
        vehicle_heading = math.radians(vehicle_transform.rotation.yaw)

        # Get the closest waypoint (lane center)
        waypoint = self.map.get_waypoint(
            vehicle_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if not waypoint:
            return 0.0, 0.0

        # Calculate lateral error (distance from vehicle to lane center)
        lane_center = waypoint.transform.location

        # Vector from lane center to vehicle
        dx = vehicle_location.x - lane_center.x
        dy = vehicle_location.y - lane_center.y

        # Lane heading
        lane_heading = math.radians(waypoint.transform.rotation.yaw)

        # Lateral error (perpendicular distance to lane)
        lateral_error = dx * math.sin(lane_heading) - dy * math.cos(lane_heading)

        # Heading error (difference between vehicle heading and lane heading)
        heading_error = self._normalize_angle(vehicle_heading - lane_heading)

        return lateral_error, heading_error

    def get_lookahead_waypoint(self):
        """
        Get a waypoint ahead of the vehicle for predictive control
        """
        vehicle_location = self.vehicle.get_transform().location
        current_waypoint = self.map.get_waypoint(
            vehicle_location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )

        if not current_waypoint:
            return None

        # Get waypoint ahead
        future_waypoints = current_waypoint.next(self.lookahead_distance)

        if future_waypoints:
            return future_waypoints[0]
        return current_waypoint

    def compute_steering(self, dt=0.05):
        """
        Compute steering command using PID control

        Args:
            dt: Time step (seconds)

        Returns:
            steering: Steering command [-1.0, 1.0]
        """
        # Get current errors
        lateral_error, heading_error = self.get_lateral_error()

        # PID computation
        # P term: proportional to lateral error
        p_term = self.Kp * lateral_error

        # I term: integral of lateral error
        self.lateral_error_integral += lateral_error * dt
        i_term = self.Ki * self.lateral_error_integral

        # D term: rate of change of lateral error
        lateral_error_derivative = (lateral_error - self.previous_lateral_error) / dt
        d_term = self.Kd * lateral_error_derivative

        # Update previous error
        self.previous_lateral_error = lateral_error

        # Combine PID terms with heading error
        steering = -(p_term + i_term + d_term + 0.5 * heading_error)

        # Clamp steering to [-1, 1]
        steering = np.clip(steering, -1.0, 1.0)

        return steering

    def compute_throttle_brake(self):
        """
        Simple speed controller

        Returns:
            throttle, brake: Control commands [0.0, 1.0]
        """
        # Get current speed
        velocity = self.vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)

        # Speed error
        speed_error = self.target_speed - current_speed

        # Simple proportional control
        if speed_error > 0:
            throttle = np.clip(self.speed_Kp * speed_error, 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = np.clip(-self.speed_Kp * speed_error, 0.0, 1.0)

        return throttle, brake

    def run_step(self, dt=0.05):
        """
        Execute one control step

        Args:
            dt: Time step (seconds)

        Returns:
            control: carla.VehicleControl object
        """
        # Compute control commands
        steering = self.compute_steering(dt)
        throttle, brake = self.compute_throttle_brake()

        # Create control command
        control = carla.VehicleControl()
        control.steer = steering
        control.throttle = throttle
        control.brake = brake
        control.hand_brake = False
        control.manual_gear_shift = False

        return control

    def get_metrics(self):
        """
        Get current performance metrics

        Returns:
            dict: Performance metrics
        """
        lateral_error, heading_error = self.get_lateral_error()
        velocity = self.vehicle.get_velocity()
        current_speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) * 3.6

        return {
            'lateral_error': lateral_error,
            'heading_error': math.degrees(heading_error),
            'current_speed': current_speed,
            'target_speed': self.target_speed * 3.6
        }

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

