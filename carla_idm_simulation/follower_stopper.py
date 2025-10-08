class FollowerStopperController:
    def __init__(self, U=7.5):
        """
        Initialize the FollowerStopper controller.

        Parameters:
        - U: desired cruising speed (m/s)
        """
        self.U = U  # target speed

        # Region intercepts (in meters)
        self.x0_1 = 4.5
        self.x0_2 = 5.25
        self.x0_3 = 6.0

        # Deceleration rates (in m/s^2)
        self.d1 = 1.5
        self.d2 = 1.0
        self.d3 = 0.5

    def compute_region_boundaries(self, delta_v):
        """
        Compute dynamic region boundaries based on velocity difference.
        """
        delta_v_minus = min(delta_v, 0)  # only consider catching up
        print(f"Delta V minus ______ {delta_v_minus}")
        x1 = self.x0_1 + (1/2*self.d1) * delta_v_minus ** 2
        # x1 = self.x0_1 + 0.5 / self.d1 * delta_v_minus ** 2
        x2 = self.x0_2 + (1/2*self.d2) * delta_v_minus ** 2
        x3 = self.x0_3 + (1/2*self.d3) * delta_v_minus ** 2
        return x1, x2, x3

    def compute_command_velocity(self, v_AV, v_lead, delta_x):
        """
        Compute the commanded velocity based on current state.

        Parameters:
        - v_AV: current AV velocity (m/s)
        - v_lead: lead vehicle velocity (m/s)
        - delta_x: gap between AV and lead vehicle (m)

        Returns:
        - v_cmd: commanded velocity (m/s)
        """
        delta_v = v_lead - v_AV
        x1, x2, x3 = self.compute_region_boundaries(delta_v)

        # Adjust lead velocity v = min(max(v_lead, 0), U)
        # print(f"Leader velocity: {v_lead}")
        # print(f"Follower Velocity: {v_AV}")
        v = min(max(v_lead, 0), self.U)

        # Compute commanded velocity based on region
        if delta_x <= x1:
            v_cmd = 0.0
            print(f"Zero Velocity")
        elif delta_x>x1 and delta_x <= x2:
            v_cmd = v * (delta_x - x1) / (x2 - x1)
            print(f"Adaption Region 1: {v_cmd}")
        elif delta_x>x2 and delta_x <= x3:
            v_cmd = v + (self.U - v) * (delta_x - x2) / (x3 - x2)
            print(f"Adaption Region 2: {v_cmd}")
        elif delta_x> x3:
            v_cmd = self.U
            print(f"Safe Region : {v_cmd}")
            print(f"x 3 --- {x3}")
            print(f"Delta X ******* {delta_x}")

        # if v_cmd<self.U:
        #     return v_cmd
        # else:
        #     return v_AV
        return v_cmd

if __name__ == "__main__":
    controller = FollowerStopperController(U=7.5)

    # Example: AV going 7 m/s, lead car at 5 m/s, gap = 8.0 m
    v_AV = 7.0
    v_lead = 5.0
    delta_x = 8.0

    v_cmd = controller.compute_command_velocity(v_AV, v_lead, delta_x)
    print(f"Commanded velocity: {v_cmd:.2f} m/s")
