class IDMController:
    def __init__(self, desired_gap=10.0, max_accel=1.0, desired_speed=15.0):
        self.desired_gap = desired_gap
        self.max_accel = max_accel
        self.desired_speed = desired_speed

    def step(self, dx, dv):
        # Basic IDM formula (simplified)
        time_headway = 1.5
        accel = self.max_accel * (1 - (dv / self.desired_speed)**2 - (self.desired_gap / (dx + 1e-5))**2)
        accel = max(min(accel, 1.0), -1.0)

        if accel >= 0:
            return accel, 0.0  # throttle, brake
        else:
            return 0.0, -accel  # throttle, brake
