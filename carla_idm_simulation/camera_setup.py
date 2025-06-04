# Add these at the top of your script
import carla

class SmoothCamera:
    def __init__(self, world, follower, smoothing_factor=0.3):
        self.world = world
        self.follower = follower
        self.smoothing_factor = smoothing_factor
        self.last_transform = follower.get_transform()

    def update(self):
        current_transform = self.follower.get_transform()
        # Interpolate position
        smoothed_location = carla.Location(
            x=self.last_transform.location.x * (
                        1 - self.smoothing_factor) + current_transform.location.x * self.smoothing_factor,
            y=self.last_transform.location.y * (
                        1 - self.smoothing_factor) + current_transform.location.y * self.smoothing_factor,
            z=self.last_transform.location.z * (
                        1 - self.smoothing_factor) + current_transform.location.z * self.smoothing_factor
        )
        # Interpolate rotation (quaternion slerp would be better)
        smoothed_rotation = carla.Rotation(
            pitch=self.last_transform.rotation.pitch * (
                        1 - self.smoothing_factor) + current_transform.rotation.pitch * self.smoothing_factor,
            yaw=self.last_transform.rotation.yaw * (
                        1 - self.smoothing_factor) + current_transform.rotation.yaw * self.smoothing_factor,
            roll=self.last_transform.rotation.roll * (
                        1 - self.smoothing_factor) + current_transform.rotation.roll * self.smoothing_factor
        )
        self.last_transform = carla.Transform(smoothed_location, smoothed_rotation)

        # Apply smoothed transform to spectator
        back_vector = self.last_transform.get_forward_vector() * -8
        up_vector = carla.Location(z=3)
        camera_location = smoothed_location + back_vector + up_vector
        self.world.get_spectator().set_transform(carla.Transform(camera_location, smoothed_rotation))