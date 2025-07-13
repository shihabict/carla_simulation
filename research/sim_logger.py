import pandas as pd
import numpy as np
import os
import matplotlib

# from research.settings import EXP_NAME
from settings import BASE_DIR

matplotlib.use('Agg')
import matplotlib.pyplot as plt

class SimulationLogger:
    def __init__(self,controller_type, num_vehicle):
        self.records = []
        self.controller_type = controller_type
        self.num_vehicle = num_vehicle
        self.data_path = f"{BASE_DIR}/Reports/sim_data_g2a26cb25_fs_FS_nV_6.csv"
        self.custom_colors = [
            '#1f77b4',  # blue
            '#ff7f0e',  # orange
            '#2ca02c',  # green
            '#d62728',  # red
            '#9467bd',  # purple
            '#8c564b',  # brown
            '#e377c2',  # pink
            '#bcbd22',  # yellow-green
        ]

    def log(self, sim_time, name, location, velocity, acceleration, gap=None, command_velocity=None, reference_speed=None):
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        self.records.append({
            'time': sim_time,
            'name': name,
            'x': location.x,
            'y': location.y,
            'z': location.z,
            'speed': speed,
            'acc': acceleration,
            'gap': gap,
            'command_velocity': command_velocity,
            'reference_speed':reference_speed
        })

    def save(self):
        filename = f'sim_data_{self.controller_type}_nV_{self.num_vehicle}.csv'
        df = pd.DataFrame(self.records)
        os.makedirs('Reports', exist_ok=True)
        path = os.path.join('Reports', filename)
        df.to_csv(path, index=False)
        print(f"[Logged] Simulation data saved to {path}")

    def load_data(self,filename):
        df = pd.read_csv(filename)
        return df

    def plot_trajectories(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        plt.figure(figsize=(10, 6))
        for label, group in df.groupby('name'):
            plt.plot(group['x'], group['y'], label=label, linewidth=1)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Vehicle Trajectories')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/trajectories_{self.controller_type}_nV_{self.num_vehicle}.png')
        print(f"[Plotted] Trajectories saved to Reports/trajectories_{self.controller_type}_nV_{self.num_vehicle}.png")

    def plot_speeds(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))

        # for label, group in df.groupby('name'):
        #     plt.plot(group['time'], group['speed'], label=label, )
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]  # wrap around if more vehicles
            plt.plot(group['time'], group['speed'], label=label, color=color, linewidth=1)

        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.title('Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/speed_vs_time__{self.controller_type}_nV_{self.num_vehicle}.png')
        print(f"[Plotted] Speed profile saved to Reports/speed_vs_time_{self.controller_type}_nV_{self.num_vehicle}.png")

    def plot_reference_velocity(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)

        # df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['reference_speed'], label=label, color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Ref Speed (m/s)')
        plt.title('Reference speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/ref_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle}.png')
        print(
            f"[Plotted] Reference Velocity saved to Reports/ref_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle}.png")

    def plot_command_velocity(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['command_velocity'], label=label, color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Command Speed (m/s)')
        plt.title('Command Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/com_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle}.png')
        print(
            f"[Plotted] Command Velocity saved to Reports/com_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle}.png")

    def plot_gap_vs_time(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        followers = df[df['name'].str.contains('follower')]
        plt.figure(figsize=(10, 6))
        for idx, (name, group) in enumerate(followers.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['gap'], label=name,color=color, linewidth=1)

        plt.xlabel('Time (s)')
        plt.ylabel('Gap to Leader (m)')
        plt.title(f'Gap Between Followers and Their Leaders Over Time {self.controller_type}')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/gap_vs_time_gap_{self.controller_type}_nV_{self.num_vehicle}.png')

    def plot_acceleration(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        followers = df[df['name'].str.contains('follower')]
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['acc'], label=label, color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s)')
        plt.title('Acceleration vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/acc_vs_time_{EXP_NAME}_{self.controller_type}_nV_{self.num_vehicle}.png')
        print(f"[Plotted] Acceleration profile saved to Reports/acc_vs_time{EXP_NAME}_{self.controller_type}_nV_{self.num_vehicle}.png")


if __name__ == '__main__':
    controller_type = "FS"
    num_vehicle = 6
    sim_logger = SimulationLogger(controller_type,num_vehicle)
    sim_logger.plot_trajectories()
    sim_logger.plot_speeds()
    sim_logger.plot_reference_velocity()
    sim_logger.plot_command_velocity()
    sim_logger.plot_gap_vs_time()