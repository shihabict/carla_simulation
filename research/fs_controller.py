import numpy as np

class FollowerStopperController:
    def __init__(self, dx_min=4.5, dx_activate=6.0, decel_bands=[1.5, 1.0, 0.5]):
        self.dx_min = dx_min
        self.dx_activate = dx_activate
        self.decel = decel_bands

    def compute_velocity_command(self, r, dx, dv, v_AV):
        """
        Compute the commanded velocity using FollowerStopper logic.

        Parameters:
        - r: desired (reference) velocity [m/s]
        - dx: gap to the leader [m]
        - dv: relative velocity (v_lead - v_AV) [m/s]
        - v_AV: current velocity of the AV (ego vehicle) [m/s]
        """


        dx_mid = (self.dx_min + self.dx_activate) / 2.0
        v_lead = v_AV + dv
        v_lead = max(v_lead, 0.0)  # lead vehicle can't go backward
        v = min(r, v_lead)         # desired speed capped by leader's velocity
        # v = v_lead

        # Clamp dv to ≤ 0 (no positive closing rates allowed)
        dv = min(dv, 0)

        # Band boundaries
        dx1 = self.dx_min + (1/(2*self.decel[0])) * dv**2
        # dx1 = self.dx_min + (dv ** 2) / (2 * self.decel[0])
        dx2 = dx_mid + (1/(2*self.decel[1])) * dv**2
        # dx2 = dx_mid + (dv ** 2) / (2 * self.decel[1])
        dx3 = self.dx_activate + (1/(2*self.decel[2])) * dv**2
        # dx3 = self.dx_activate + (dv ** 2) / (2 * self.decel[2])
        # print(f"dx1: {dx1}")
        # print(f"dx2: {dx2}")
        # print(f"dx3: {dx3}")
        # print(f'----------------------------------------------------------------')

        # Piecewise logic
        if dx < dx1:
            u_cmd = 0.0
        elif dx1 <= dx < dx2:
            u_cmd = v * (dx - dx1) / (dx2 - dx1)
        elif dx2 <= dx < dx3:
            u_cmd = v + (r - v) * (dx - dx2) / (dx3 - dx2)
        else:
            u_cmd = r

        return u_cmd, (dx1,dx2,dx3)

# if __name__ == '__main__':
#     controller = FollowerStopperController()
#     cmd_velocity = controller.compute_velocity_command(
#         r=25.0,  # reference speed
#         dx=6.5,  # spacing to leader
#         dv=-2.0,  # leader is slower
#         v_AV=10.0  # current ego speed
#     )
#     print(f"Commanded velocity: {cmd_velocity:.2f} m/s")
