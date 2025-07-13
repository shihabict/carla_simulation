class NominalController:
    def __init__(self,dt):
        self.y = 0.0  # Persistent speed value across calls
        self.dt = dt  # Fixed timestep, matching CARLA's fixed_delta_seconds

    def get_reference_speed(self,vel, max_speed=20, max_accel=1.5, max_decel=1.5):
        if self.y > max_speed + 1:
            self.y = max(max_speed, self.y - abs(max_decel) * self.dt)
        elif self.y < max_speed - 1:
            self.y = min(max_speed, self.y + max_accel * self.dt)
        else:
            self.y = float(max_speed)

        # Safety clamp to ensure y does not get too low
        if self.y < 2 and max_speed > 2:
            self.y = 2
        elif self.y < 1 and max_speed > 1:
            self.y = 1

        # Final reference velocity bounded between vel-1 and vel+2
        r = min(max(self.y, vel - 1.0), vel + 2.0)
        return r
