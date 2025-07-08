import pandas as pd
import numpy as np
import os

class SimulationLogger:
    def __init__(self):
        self.records = []

    def log(self, timestamp, label, location, velocity):
        speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
        self.records.append({
            'time': timestamp,
            'vehicle': label,
            'x': location.x,
            'y': location.y,
            'speed': speed
        })

    def save(self, filename='sim_data.csv'):
        df = pd.DataFrame(self.records)
        os.makedirs('Reports', exist_ok=True)
        path = os.path.join('Reports', filename)
        df.to_csv(path, index=False)
        print(f"[Logged] Simulation data saved to {path}")

    def plot_trajectories(self):
        import matplotlib.pyplot as plt
        df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))
        for label, group in df.groupby('vehicle'):
            plt.plot(group['x'], group['y'], label=label)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Vehicle Trajectories')
        plt.grid()
        plt.legend()
        plt.savefig('Reports/trajectories.png')
        print("[Plotted] Trajectories saved to Reports/trajectories.png")

    def plot_speeds(self):
        import matplotlib.pyplot as plt
        df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))
        for label, group in df.groupby('vehicle'):
            plt.plot(group['time'], group['speed'], label=label)
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.title('Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig('Reports/speed_vs_time.png')
        print("[Plotted] Speed profile saved to Reports/speed_vs_time.png")
