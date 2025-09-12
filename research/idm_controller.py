import numpy as np

class IDMController:
    def __init__(self, desired_speed=25.0, max_accel=3, comfortable_brake=1.5,
                 min_gap=3.4, time_headway=2, delta=4):
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
        # if ego_v < 0.1 and gap > self.s0 + 1.5 and lead_v > 0.3:
        #     return 0.5  # Force a gentle restart
        delta_v = ego_v - lead_v
        s_star = self.desired_gap(ego_v, delta_v)
        # s_star = min(s_star, gap * 2)  # avoid s*/s >> 1


        # acc = self.a_max * (1 - (ego_v / self.v0) ** self.delta - (s_star / gap) ** 2) if gap > 0.1 else -self.b
        acc = self.a_max * (1 - (ego_v / self.v0) ** self.delta - (s_star / gap) ** 2)
        clipped_acc = np.clip(acc, -self.b, self.a_max)
        # print(f"ACC {acc} ---- clipped {clipped_acc}")
        return clipped_acc
        # return max(min(acc, 1.0), -1.0)

