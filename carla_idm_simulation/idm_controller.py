# class IDMController:
#     def __init__(self, desired_gap=10.0, max_accel=1.0, desired_speed=15.0):
#         self.desired_gap = desired_gap
#         self.max_accel = max_accel
#         self.desired_speed = desired_speed
#
#
#     def step(self, dx, dv):
#         effective_desired_speed = self.desired_speed
#
#         accel = self.max_accel * (1 - (dv / effective_desired_speed) ** 2 -
#                                   (self.desired_gap / (dx + 1e-5)) ** 2)
#         accel = max(min(accel, 1.0), -1.0)
#
#         return (accel, 0.0) if accel >= 0 else (0.0, -accel)


class IDMController:
    def __init__(self, desired_speed=15.0, desired_gap=10.0, time_headway=1.5,
                 max_accel=1.0, comfortable_brake=1.5, min_spacing=2.0):
        self.v0 = desired_speed
        self.s0 = min_spacing
        self.T = time_headway
        self.a_max = max_accel
        self.b = comfortable_brake

    def step(self, dx, ego_v, lead_v):
        dv = ego_v - lead_v
        s_star = self.s0 + ego_v * self.T + (ego_v * dv) / (2 * (self.a_max * self.b)**0.5)
        accel = self.a_max * (1 - (ego_v / self.v0)**4 - (s_star / (dx + 1e-5))**2)
        return max(min(accel, 1.0), -1.0)
        # return accel
