import numpy as np

class IDMController:
    def __init__(self, desired_speed=20.0, max_accel=2.0, comfortable_brake=1.5,
                 min_gap=2.5, time_headway=2, delta=2):
        """
        desired_speed: v0 (m/s)
        max_accel: a_max (m/s^2)
        comfortable_brake: b (m/s^2)
        min_gap: s0 (m)
        time_headway: T (s)
        delta: exponent
        """
        self.v0 = desired_speed
        self.a_max = max_accel
        self.b = comfortable_brake
        self.s0 = min_gap
        self.T = time_headway
        self.delta = delta

    def desired_gap(self, ego_v, delta_v):
        """s* = s0 + v * T + v * delta_v / (2 * sqrt(ab))"""
        sqrt_ab = np.sqrt(self.a_max * self.b)
        return self.s0 + ego_v * self.T + (ego_v * delta_v) / (2 * sqrt_ab)

    def compute_acceleration(self, ego_v, lead_v, gap):
        """
        ego_v: ego vehicle speed (m/s)
        lead_v: leader vehicle speed (m/s)
        gap: distance to leader (m)
        """
        delta_v = ego_v - lead_v
        s_star = self.desired_gap(ego_v, delta_v)

        acc = self.a_max * (1 - (ego_v / self.v0) ** self.delta - (s_star / gap) ** 2) if gap > 0.1 else -self.b
        return np.clip(acc, -self.b, self.a_max)
