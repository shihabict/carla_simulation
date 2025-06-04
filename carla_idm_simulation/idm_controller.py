class IDMController:
    def __init__(self, desired_gap=10.0, max_accel=1.0, desired_speed=15.0):
        self.desired_gap = desired_gap
        self.max_accel = max_accel
        self.desired_speed = desired_speed


    def step(self, dx, dv, curvature=0):
        # Add curvature term to reduce speed in turns
        turn_factor = 1.0 / (1.0 + 5.0 * abs(curvature))
        effective_desired_speed = self.desired_speed * turn_factor

        accel = self.max_accel * (1 - (dv / effective_desired_speed) ** 4 -
                                  (self.desired_gap / (dx + 1e-5)) ** 2)
        accel = max(min(accel, 1.0), -1.0)

        return (accel, 0.0) if accel >= 0 else (0.0, -accel)
