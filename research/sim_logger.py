import pandas as pd
import numpy as np
import os
import matplotlib

from research.settings import EXP_NAME

matplotlib.use('Agg')
import matplotlib.pyplot as plt

class SimulationLogger:
    def __init__(self):
        self.records = []

    def log(self, sim_time, name, location, velocity, gap=None):
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        self.records.append({
            'time': sim_time,
            'name': name,
            'x': location.x,
            'y': location.y,
            'z': location.z,
            'speed': speed,
            'gap': gap
        })

    def save(self, filename='sim_data.csv'):
        df = pd.DataFrame(self.records)
        os.makedirs('Reports', exist_ok=True)
        path = os.path.join('Reports', filename)
        df.to_csv(path, index=False)
        print(f"[Logged] Simulation data saved to {path}")

    def plot_trajectories(self):

        df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))
        for label, group in df.groupby('name'):
            plt.plot(group['x'], group['y'], label=label)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Vehicle Trajectories')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/trajectories_{EXP_NAME}.png')
        print("[Plotted] Trajectories saved to Reports/trajectories.png")

    def plot_speeds(self):
        df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))
        for label, group in df.groupby('name'):
            plt.plot(group['time'], group['speed'], label=label, )
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.title('Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/speed_vs_time_{EXP_NAME}.png')
        print(f"[Plotted] Speed profile saved to Reports/speed_vs_time{EXP_NAME}.png")

    def plot_gap_vs_time(self):
        df = pd.DataFrame(self.records)
        followers = df[df['name'].str.contains('follower')]
        plt.figure(figsize=(10, 6))
        for name, group in followers.groupby('name'):
            plt.plot(group['time'], group['gap'], label=name)

        plt.xlabel('Time (s)')
        plt.ylabel('Gap to Leader (m)')
        plt.title('Gap Between Followers and Their Leaders Over Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Reports/gap_vs_time_gap_{EXP_NAME}.png')
