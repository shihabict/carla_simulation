import pandas as pd
import numpy as np
import os
import matplotlib

# from research.settings import EXP_NAME
from settings import BASE_DIR

matplotlib.use('Agg')
import matplotlib.pyplot as plt

class SimulationLogger:
    def __init__(self,controller_type, num_vehicle, reference_speed, sampling_frequency, switch_time):
        self.records = []
        self.controller_type = controller_type
        self.num_vehicle = num_vehicle
        self.data_path = f"{BASE_DIR}/Final_Reports/sim_data_FS_IDM_avg_ref_speed_nV_6_ref25_f0.02.csv"
        self.reference_speed = reference_speed
        self.sampling_frequency = sampling_frequency
        self.switch_time = switch_time
        # self.custom_colors = [
        #     "#041e31",
        #     '#ff7f0e',
        #     '#2ca02c',
        #     '#d62728',
        #     "#7d49ad",
        #     '#8c564b',
        #     "#eeb1dc",
        #     '#bcbd22',
        # ]

        self.custom_colors = ['#38028F', '#8F0202', '#8F0244', '#02448F', '#028F4A', '#8F6A02', "#00B1B8"]

    def log(self, sim_time, name, location, velocity, acceleration, gap=None, ref_speed=None, rel_speed=None):
        # speed = np.linalg.norm([velocity.x])
        self.records.append({
            'time': sim_time,
            'name': name,
            'x': location.x,
            'y': location.y,
            'z': location.z,
            'speed': velocity,
            'acc': acceleration,
            'gap': gap,
            'ref_velocity': ref_speed,
            'rel_velocity': rel_speed
        })
        # print(0)

    def save(self):
        filename = f'sim_data_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.csv'
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
        if self.switch_time:
            plt.title(f'Vehicle Trajectories with reference speed {self.reference_speed} and IDM({self.switch_time})')
        else:
            plt.title(f'Vehicle Trajectories with reference speed {self.reference_speed}')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/trajectories_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf')
        print(f"[Plotted] Trajectories saved to Final_Reports/trajectories_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_speeds(self,start_time=None,end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        plt.figure(figsize=(10, 6))

        # for label, group in df.groupby('name'):
        #     plt.plot(group['time'], group['speed'], label=label, )
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]  # wrap around if more vehicles
            plt.plot(group['time'], group['speed'], label=f"Vehicle {idx}", color=color, linewidth=1)

        plt.xlabel('Time (s)')
        plt.ylabel('Speed (m/s)')
        plt.title(f'Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/speed_vs_time__{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)
        print(f"[Plotted] Speed profile saved to Final_Reports/speed_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_reference_velocity(self,start_time=None,end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        # df = pd.DataFrame(self.records)
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['ref_velocity'], label=f"Vehicle {idx}", color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Ref Speed (m/s)')
        plt.title(f'Reference speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/ref_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)
        print(
            f"[Plotted] Reference Velocity saved to Final_Reports/ref_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_relative_velocity(self,start_time=None, end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        df = df[df['name']!='leader']
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['rel_velocity'], label=f"Vehicle {idx}", color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Relative Velocity (m/s)')
        plt.title(f'Relative Velocity vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/rel_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)
        print(
            f"[Plotted] Relative Velocity saved to Final_Reports/rel_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")



    def plot_command_velocity(self, start_time=None, end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['command_velocity'], label=f"Vehicle {idx}", color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Command Speed (m/s)')
        plt.title(f'Command Speed vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/com_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)
        print(
            f"[Plotted] Command Velocity saved to Final_Reports/com_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_gap_vs_time(self,start_time=None, end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        followers = df[df['name'].str.contains('car')]
        plt.figure(figsize=(10, 6))
        for idx, (name, group) in enumerate(followers.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['gap'], label=f"Vehicle {idx}", color=color, linewidth=1)

        plt.xlabel('Time (s)')
        plt.ylabel('Space Headway (m)')
        plt.title(f'Space Headway Vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/space_headway_vs_time_gap_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)

    def plot_acceleration(self,start_time=None, end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        followers = df[df['name'].str.contains('car')]
        plt.figure(figsize=(10, 6))
        for idx, (label, group) in enumerate(df.groupby('name')):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['acc'], label=f"Vehicle {idx}", color=color, linewidth=1)
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s)')
        plt.title(f'Acceleration vs Time')
        plt.grid()
        plt.legend()
        plt.savefig(f'Final_Reports/acc_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)
        print(f"[Plotted] Acceleration profile saved to Final_Reports/acc_vs_timeE_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_relative_speeds(self, x_col, y_col, title, start_time=None, end_time=None):

        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        # # Filter data by time window
        # df_filtered = df[(df[x_col] >= start_time) & (df[x_col] <= end_time)]

        # Pivot data to wide format: time as index, name as columns
        pivot_df = df.pivot(index=x_col, columns='name', values=y_col).dropna()

        # Sort columns so followers are in order
        vehicle_names = sorted(pivot_df.columns, key=lambda x: (x != 'leader', x))

        # Plot relative speed
        plt.figure(figsize=(12, 6))
        for i in range(1, len(vehicle_names)):
            color = self.custom_colors[i % len(self.custom_colors)]
            leader = vehicle_names[i - 1]
            follower = vehicle_names[i]
            rel_speed = pivot_df[leader] - pivot_df[follower]
            plt.plot(pivot_df.index, rel_speed, label=f'Δv_{i}', color=color)

        plt.xlabel(x_col)
        plt.ylabel('Relative Speed (m/s)')
        plt.title(title)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()
        plt.savefig(f'Final_Reports/rel_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)

    def plot_time_space_diagram(self, start_time=None, end_time=None):

        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)


        x_max = df['x'].max()
        x_min = df['x'].min()
        df['x'] = x_max - df['x']

        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)

            print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
            df_filtered = df.iloc[lower_limit:upper_limit]
            df = df_filtered

        # Plot
        plt.figure(figsize=(12, 6))
        i = 1
        for vehicle_name in df['name'].unique():
            vehicle_df = df[df['name'] == vehicle_name]
            plt.plot(vehicle_df['time'], vehicle_df['x'], label=f'Vehicle {i}')
            i += 1

        plt.xlabel("Time (s)")
        plt.ylabel("Longitudinal Position (x in meters)")
        plt.title("Time-Space Diagram")
        plt.legend(loc='upper right', ncol=2)
        plt.grid(True)
        plt.tight_layout()
        plt.show()
        plt.savefig(
            f'Final_Reports/time_space_{self.controller_type}_nV_{self.num_vehicle + 2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)

    # def plot_time_space_diagram(self, start_time=None, end_time=None):
    #     # Load data
    #     df = pd.read_csv(csv_path)
    #     x_max = df['x'].max()
    #     x_min = df['x'].min()
    #     df['x'] = x_max - df['x']  # Remaps [x_min, x_max] to [x_max - x_min, 0]
    #
    #     # Optional time filtering
    #     if start_time is not None:
    #         df = df[df['time'] >= start_time]
    #     if end_time is not None:
    #         df = df[df['time'] <= end_time]
    #
    #     # Plot
    #     plt.figure(figsize=(12, 6))
    #     for vehicle_name in df['name'].unique():
    #         vehicle_df = df[df['name'] == vehicle_name]
    #         plt.plot(vehicle_df['time'], vehicle_df['x'], label=vehicle_name)
    #
    #     plt.xlabel("Time (s)")
    #     plt.ylabel("Longitudinal Position (x in meters)")
    #     plt.title("Time-Space Diagram")
    #     plt.legend(loc='upper right', ncol=2)
    #     plt.grid(True)
    #     plt.tight_layout()
    #     plt.show()

if __name__ == '__main__':
    controller_type = "FS_IDM"
    num_vehicle = 6
    sim_logger = SimulationLogger(controller_type,num_vehicle,reference_speed=25, sampling_frequency=0.02, switch_time=120)
    sim_logger.plot_speeds(start_time=0, end_time=2600)
    sim_logger.plot_reference_velocity(start_time=0, end_time=2600)
    sim_logger.plot_gap_vs_time(start_time=0, end_time=2600)
    sim_logger.plot_relative_speeds('time', 'speed', 'Relative Speed vs Time', start_time=0, end_time=2600)
    sim_logger.plot_time_space_diagram(start_time=0, end_time=2600)