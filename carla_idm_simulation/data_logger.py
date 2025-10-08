import csv
import os
import time
import math


class DataLogger:
    def __init__(self, filename="car_following_data.csv",num_followers=1):
        self.filename = filename
        self.num_followers = num_followers
        self.file = None
        self.writer = None
        self.start_time = time.time()

        # Create directory if doesn't exist
        os.makedirs("logs", exist_ok=True)
        self.filepath = os.path.join("logs", self.filename)

    def start_logging(self):
        # headers = [
        #     "timestamp",
        #     # Leader data
        #     "leader_x", "leader_y", "leader_z",
        #     "leader_speed", "leader_throttle", "leader_brake", "leader_steer",
        #     # Follower data
        #     "follower_x", "follower_y", "follower_z",
        #     "follower_speed", "follower_throttle", "follower_brake", "follower_steer",
        #     "distance",
        #     "relative_speed"
        # ]
        headers = [
            "timestamp",
            "leader_x", "leader_y", "leader_z",
            "leader_speed", "leader_throttle", "leader_brake", "leader_steer"
        ]

        # Add fields for each follower
        for i in range(self.num_followers):
            headers += [
                f"f{i + 1}_x", f"f{i + 1}_y", f"f{i + 1}_z",
                f"f{i + 1}_speed", f"f{i + 1}_throttle", f"f{i + 1}_brake", f"f{i + 1}_steer",
                f"f{i + 1}_distance", f"f{i + 1}_relative_speed"
            ]

        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=headers)
        self.writer.writeheader()

    # def log_data(self, leader, followers):
    #     if not self.file:
    #         self.start_logging()
    #
    #     leader_control = leader.get_control()
    #     follower_control = follower.get_control()
    #
    #     leader_vel = leader.get_velocity()
    #     follower_vel = follower.get_velocity()
    #
    #     data = {
    #         "timestamp": time.time() - self.start_time,
    #         # Leader data
    #         "leader_x": leader.get_transform().location.x,
    #         "leader_y": leader.get_transform().location.y,
    #         "leader_z": leader.get_transform().location.z,
    #         "leader_speed": math.sqrt(leader_vel.x ** 2 + leader_vel.y ** 2),
    #         "leader_throttle": leader_control.throttle,
    #         "leader_brake": leader_control.brake,
    #         "leader_steer": leader_control.steer,
    #         # Follower data
    #         "follower_x": follower.get_transform().location.x,
    #         "follower_y": follower.get_transform().location.y,
    #         "follower_z": follower.get_transform().location.z,
    #         "follower_speed": math.sqrt(follower_vel.x ** 2 + follower_vel.y ** 2),
    #         "follower_throttle": follower_control.throttle,
    #         "follower_brake": follower_control.brake,
    #         "follower_steer": follower_control.steer,
    #         "distance": distance,
    #         "relative_speed": relative_speed
    #     }
    #
    #     self.writer.writerow(data)
    #     self.file.flush()  # Ensure data is written immediately

    def log_data(self, leader, followers):
        if not self.file:
            self.start_logging()

        leader_control = leader.get_control()
        leader_vel = leader.get_velocity()
        leader_speed = math.sqrt(leader_vel.x ** 2 + leader_vel.y ** 2)

        data = {
            "timestamp": time.time() - self.start_time,
            "leader_x": leader.get_transform().location.x,
            "leader_y": leader.get_transform().location.y,
            "leader_z": leader.get_transform().location.z,
            "leader_speed": leader_speed,
            "leader_throttle": leader_control.throttle,
            "leader_brake": leader_control.brake,
            "leader_steer": leader_control.steer
        }

        leader_loc = leader.get_transform().location

        for i, follower in enumerate(followers):
            f_control = follower.get_control()
            f_vel = follower.get_velocity()
            f_speed = math.sqrt(f_vel.x ** 2 + f_vel.y ** 2)

            f_loc = follower.get_transform().location
            distance = leader_loc.distance(f_loc)
            relative_speed = leader_speed - f_speed

            data.update({
                f"f{i + 1}_x": f_loc.x,
                f"f{i + 1}_y": f_loc.y,
                f"f{i + 1}_z": f_loc.z,
                f"f{i + 1}_speed": f_speed,
                f"f{i + 1}_throttle": f_control.throttle,
                f"f{i + 1}_brake": f_control.brake,
                f"f{i + 1}_steer": f_control.steer,
                f"f{i + 1}_distance": distance,
                f"f{i + 1}_relative_speed": relative_speed
            })

        self.writer.writerow(data)
        self.file.flush()

    def stop_logging(self):
        if self.file:
            self.file.close()