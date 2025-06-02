import math

class IDMController:
    def __init__(self, v0=20.0,    # desired velocity (m/s)
                 T=1.5,           # desired time headway (s)
                 a_max=2.0,       # maximum acceleration (m/s^2)
                 b=2.0,           # comfortable deceleration (m/s^2)
                 delta=4,         # acceleration exponent
                 s0=2.0):         # minimum spacing (m)
        self.v0 = v0
        self.T = T
        self.a_max = a_max
        self.b = b
        self.delta = delta
        self.s0 = s0

    def calculate_acceleration(self, s, v_follower, v_leader):
        """
        IDM acceleration formula.

        Args:
            s (float): gap to the leader (meters)
            v_follower (float): current speed of follower (m/s)
            v_leader (float): current speed of leader (m/s)

        Returns:
            float: acceleration command (m/s^2)
        """
        if s <= 0:
            s = 0.01  # avoid division by zero

        dv = v_follower - v_leader  # relative speed (positive = follower is faster)

        # desired dynamic spacing
        s_star = self.s0 + v_follower * self.T + (v_follower * dv) / (2 * math.sqrt(self.a_max * self.b))

        # IDM acceleration
        acc = self.a_max * (1 - (v_follower / self.v0) ** self.delta - (s_star / s) ** 2)

        return acc
