import pandas as pd
import os
import matplotlib
import numpy as np

import scipy

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams  # Correct import

# Enable LaTeX rendering
# Plot styling
axes_size = 26
tick_size = 22
rcParams["text.usetex"] = True
rcParams["font.family"] = "serif"
rcParams["font.serif"] = ["Times"]
rcParams["font.size"] = tick_size
rcParams["axes.labelsize"] = axes_size
rcParams["xtick.labelsize"] = tick_size
rcParams["ytick.labelsize"] = tick_size
rcParams["legend.fontsize"] = tick_size
from settings import BASE_DIR


class SimulationLogger:
    def __init__(self,controller_type, num_vehicle, reference_speed, sampling_frequency, switch_time):
        self.records = []
        self.controller_type = controller_type
        self.num_vehicle = num_vehicle
        if self.controller_type == "IDM":
            self.data_path = f"{BASE_DIR}/Reports/sim_data_with_distance_IDM.csv"
        else:
            self.data_path = f"{BASE_DIR}/Final_Reports/sim_data_FsIdmTesting_nV_8_ref25_f50.csv"
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
        print(df.shape)
        df.to_csv(path)
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
        plt.xlabel('X (m)',fontsize=18)
        plt.ylabel('Y (m)',fontsize=18)
        if self.switch_time:
            plt.title(f'Vehicle Trajectories with reference speed {self.reference_speed} and IDM({self.switch_time})',fontsize=20)
        else:
            plt.title(f'Vehicle Trajectories with reference speed {self.reference_speed}',fontsize=20)
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
            if idx == 0:
                plt.plot(group['time'], group['speed'], label=f"Leader", color=color, linewidth=1)
            else:
                plt.plot(group['time'], group['speed'], label=f"Vehicle {idx}", color=color, linewidth=1)

        if self.controller_type!="IDM":
            # Add vertical dashed line at switching time
            switch_time = 120  # seconds
            plt.axvline(x=switch_time, color='purple', linestyle='--', linewidth=1)

            # Add text annotations
            plt.text(switch_time - 55, plt.ylim()[1] * 0.55, "Manual Driving",
                     fontsize=20, ha='center', va='top', color='black')
            plt.text(switch_time + 85, plt.ylim()[1] * 0.55, "Mixed Autonomy",
                     fontsize=20, ha='center', va='top', color='black')
            plt.text(switch_time + 6, plt.ylim()[1] * 0.01, "FS Activation", rotation=90, color='purple', fontsize=18)
        if self.controller_type!="IDM":

            plt.title(r"\textbf{Speed Profiles Under Mixed Autonomy with FS Controller}",fontsize=20)
        else:
            plt.title(r"\textbf{Speed Profiles Under Fully Human-Driven Behavior (IDM)}", fontsize=20)

        plt.xlabel(r"\textbf{Time (s)}",fontsize=18)
        plt.ylabel(r"\textbf{Speed (m/s)}",fontsize=18)
        # plt.title(r"\textbf{Speed Over Time}")
        plt.legend(loc='lower right',ncol=2,fontsize=18)
        plt.grid(True)
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.show()
        plt.savefig(f'Final_Reports/SpeedVsTime__{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300, format="pdf", bbox_inches='tight')
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
        plt.xlabel('Time (s)',fontsize=18)
        plt.ylabel('Ref Speed (m/s)',fontsize=18)
        plt.title(f'Reference speed vs Time',fontsize=20)
        plt.grid()
        plt.legend(ncol=2)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
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
        plt.xlabel('Time (s)',fontsize=18)
        plt.ylabel('Relative Velocity (m/s)',fontsize=18)
        plt.title(f'Relative Velocity vs Time',fontsize=20)
        plt.grid(True)
        plt.legend(ncol=2)
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.savefig(f'Final_Reports/RelVelVsTime_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300, format='pdf', bbox_inches='tight')
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
        plt.xlabel('Time (s)',fontsize=18)
        plt.ylabel('Command Speed (m/s)',fontsize=18)
        plt.title(f'Command Speed vs Time',fontsize=20)
        plt.grid()
        plt.legend(ncol=True)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        plt.savefig(f'Final_Reports/com_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300)
        print(
            f"[Plotted] Command Velocity saved to Final_Reports/com_vel_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_gap_vs_time(self):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)
        # df = pd.DataFrame(self.records)

        # if start_time is not None and end_time is not None:
        #     lower_limit = int(start_time * 50)
        #     upper_limit = int(end_time * 50)
        #
        #     print(f"Plotting from step {lower_limit} to {upper_limit} (time {start_time} to {end_time})")
        #     df_filtered = df.iloc[lower_limit:upper_limit]
        #     df = df_filtered

        followers = df[df['name'].str.contains('car')]
        plt.figure(figsize=(11, 6.5))
        follower_groupby = followers.groupby('name')
        for idx, (name, group) in enumerate(follower_groupby):
            color = self.custom_colors[idx % len(self.custom_colors)]
            plt.plot(group['time'], group['gap'], label=fr'$\Delta x_{{{idx+1}}}(t)$', color=color, linewidth=1)

        # Add vertical dashed line at switching time
        switch_time = 120  # seconds
        plt.axvline(x=switch_time, color='purple', linestyle='--', linewidth=1)

        # Add text annotations
        plt.text(switch_time - 65, plt.ylim()[1] * 0.85, "Manual Driving",
                 fontsize=22, ha='center', va='top', color='black')
        plt.text(switch_time + 85, plt.ylim()[1] * 0.85, "Mixed Autonomy",
                 fontsize=22, ha='center', va='top', color='black')
        plt.text(switch_time + 6, plt.ylim()[1] * 0.2, "FS Activation", rotation=90, color='purple',fontsize=18)

        # Axis limits
        # if xlim[0] is not None or xlim[1] is not None:
        #     plt.xlim(left=xlim[0], right=xlim[1])
        # if ylim[0] is not None or ylim[1] is not None:
        #     plt.ylim(bottom=ylim[0], top=ylim[1])

        plt.xlabel(r"\textbf{Time (s)}",fontsize=18)
        plt.ylabel(r"\textbf{Space Headway (m)}",fontsize=18)
        plt.title(r"\textbf{Space Headway ($\Delta x$) Between Successive Vehicles Over Time}",fontsize=24)
        plt.xlim(0,300)
        plt.ylim(0,100)
        plt.grid(True)
        plt.legend(loc='upper right',ncol=2,fontsize=18)
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.savefig(f'Final_Reports/SpaceHeadway{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300,format='pdf',bbox_inches='tight')

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
        plt.xlabel('Time (s)',fontsize=18)
        plt.ylabel('Acceleration (m/s)',fontsize=18)
        plt.title(f'Acceleration vs Time',fontsize=20)
        plt.grid()
        plt.legend(ncol=2)
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.savefig(f'Final_Reports/acc_vs_time_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300, format='pdf', bbox_inches='tight')
        print(f"[Plotted] Acceleration profile saved to Final_Reports/acc_vs_timeE_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf")

    def plot_relative_speeds(self, x_col, y_col, title, switch_time=120, start_time=None, end_time=None):

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
        plt.figure(figsize=(11, 7))
        for i in range(1, len(vehicle_names)):
            color = self.custom_colors[i % len(self.custom_colors)]
            leader = vehicle_names[i - 1]
            follower = vehicle_names[i]
            rel_speed = pivot_df[leader] - pivot_df[follower]
            plt.plot(pivot_df.index, rel_speed, label=fr'$\Delta v_{{{i}}}(t)$', color=color)

        # Add vertical dashed line at switching time
        switch_time = 120  # seconds
        plt.axvline(x=switch_time, color='purple', linestyle='--', linewidth=1)

        # Add text annotations
        plt.text(switch_time - 85, plt.ylim()[1] * 0.85, "Manual Driving",
                 fontsize=20, ha='center', va='top', color='black')
        plt.text(switch_time + 85, plt.ylim()[1] * 0.85, "Mixed Autonomy",
                 fontsize=20, ha='center', va='top', color='black')
        plt.text(switch_time + 6, plt.ylim()[1] * -0.7, "FS Activation", rotation=90, color='purple', fontsize=18)

        plt.xlabel(r"\textbf{Time (s)}",fontsize=18)
        plt.ylabel(r"\textbf{Relative Speed (m/s)}",fontsize=18)
        plt.title(title)
        plt.grid(True)
        plt.legend(loc='lower right',ncol=2,fontsize=20)
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.tight_layout()
        plt.show()
        plt.savefig(f'Final_Reports/RelVelVsTime_{self.controller_type}_nV_{self.num_vehicle+2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300, format='pdf', bbox_inches='tight')


    def plot_time_space_diagram(self, start_time=None, end_time=None):

        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)

        df=df.loc[8:]
        df['newGap'] = df['gap'] + 4.6
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

        plt.xlabel("Time (s)",fontsize=18)
        plt.ylabel("Longitudinal Position (x in meters)",fontsize=18)
        plt.title("Time-Space Heatmap of Vehicle Speeds",fontsize=20)
        plt.legend(ncol=2,fontsize=18)
        plt.grid(True)
        plt.tight_layout()
        plt.xticks(fontsize=24)
        plt.yticks(fontsize=24)
        plt.show()
        plt.savefig(
            f'Final_Reports/time_space_{self.controller_type}_nV_{self.num_vehicle + 2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',dpi=300,format='pdf', bbox_inches='tight')

    def plot_spatiotemporal_heatmap(self, new_cmap):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)

        x_max = df['x'].max()
        # df['x'] = x_max - df['x']
        # Make a copy to avoid modifying original DataFrame

        # df['time_step'] = df.index // 8
        data = df.copy()
        # data.drop('time', axis=1, inplace=True)
        # Rename columns to standard names
        # data = data.rename(columns={
        #     'speed': 'speed',
        #     'x': 'pos',
        #     'time_step': 'time'
        # })


        # Drop NaNs and keep only nonnegative times
        data = data.dropna(subset=['speed', 'x', 'time'])
        dt_mean = np.mean(data['speed'])
        # data = data[data['time'] >= 0]
        # data['time'] = (data['time'] * 1 / 50).astype(float)  # keep as float seconds

        # Set the number of bins for the grid
        num_bins_time = 200
        num_bins_pos = 400

        # Calculate average speed in each (time, pos) bin
        h, xedges, yedges, binnumbers = scipy.stats.binned_statistic_2d(
            data['time'],
            data['x'],
            data['speed'],
            statistic='mean',
            bins=[num_bins_time, num_bins_pos]
        )

        # Calculate the overall mean speed
        overall_mean_speed = np.nanmean(data['speed'])
        print("Overall mean speed:", overall_mean_speed)

        # Prepare colormap with white for missing data
        cmap = new_cmap.with_extremes(bad='white')

        # Plot heatmap
        plt.figure(figsize=(10, 4))
        # plt.ylim(data['x'].min(), 2000)
        # Important fix: use bin limits for axes
        plt.xlim(xedges[0], xedges[-1])
        plt.ylim(yedges[0], 8000)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)

        pcm = plt.pcolormesh(xedges, yedges, h.T, cmap=cmap,
                             rasterized=True, vmin=0, vmax=33, shading="auto")

        cbar = plt.colorbar(pcm, label="Speed (m/s)", pad=0)
        cbar.set_label(r'\textbf{Speed (m/s)}', fontsize=18)
        cbar.ax.tick_params(labelsize=20)

        plt.xlabel(r'\textbf{Time (s)}', fontsize=18)
        plt.ylabel(r'\textbf{Longitudinal Position (m)}', fontsize=18)
        plt.title(r"\textbf{Time–Space Heatmap of Vehicle Speeds}", fontsize=20)
        plt.savefig(
            f'Final_Reports/timeSpaceHeatmap_{self.controller_type}_nV_{self.num_vehicle + 2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',
            dpi=300,format="pdf",bbox_inches='tight')

        # if save_pdf:
        #     if save_path is None:
        #         save_path = f"NC_plot_r{ref_speed}.pdf"
        #     os.makedirs(os.path.dirname(save_path), exist_ok=True)
        #     plt.savefig(save_path, dpi=dpi, format="pdf", bbox_inches='tight')
        #     print(f"Plot saved to: {save_path}")

        plt.show()

    def plot_spatiotemporal_diagram(self, start_time=None, end_time=None):
        if self.records:
            df = pd.DataFrame(self.records)
        else:
            df = self.load_data(self.data_path)

        # Preprocess
        df = df.loc[8:]
        df['newGap'] = df['gap'] + 4.6
        x_max = df['x'].max()
        df['x'] = x_max - df['x']  # Flip position

        # Apply time filter
        if start_time is not None and end_time is not None:
            lower_limit = int(start_time / 0.02)
            upper_limit = int(end_time / 0.02)
            df = df.iloc[lower_limit:upper_limit]

        # Pivot data to create time vs position grid
        pivot_df = df.pivot(index='time', columns='name', values='speed')

        # Interpolate missing values
        pivot_df = pivot_df.interpolate(method='linear', axis=0).fillna(0)

        # Create X and Y grid (time and vehicle position index)
        time_grid = pivot_df.index.values
        vehicle_index = np.arange(len(pivot_df.columns))

        # Build meshgrid
        T, V = np.meshgrid(time_grid, vehicle_index)

        # Values: speeds (normalize across vehicles)
        speed_values = pivot_df.values.T  # Transpose so vehicles align with axis

        # Plot heatmap
        plt.figure(figsize=(12, 6))
        cmap = plt.get_cmap('viridis')

        plt.pcolormesh(T, V, speed_values, shading='auto', cmap=cmap)
        plt.colorbar(label='Speed (m/s)')
        plt.xlabel('Time (s)', fontsize=18)
        plt.ylabel('Vehicle Index', fontsize=18)
        plt.title('Spatiotemporal Diagram of Speeds', fontsize=20)
        plt.xticks(fontsize=18)
        plt.yticks(fontsize=18)
        plt.grid(False)
        plt.tight_layout()
        plt.show()

        # Save figure
        plt.savefig(
            f'Final_Reports/spatiotemporal_{self.controller_type}_nV_{self.num_vehicle + 2}_ref{self.reference_speed}_f{self.sampling_frequency}.pdf',
            dpi=300, format='pdf', bbox_inches='tight'
        )

if __name__ == '__main__':
    controller_type = "FsIdmTesting"
    num_vehicle = 6
    sim_logger = SimulationLogger(controller_type,num_vehicle,reference_speed=25, sampling_frequency=0.02, switch_time=120)
    sim_logger.plot_speeds(start_time=0, end_time=2600)
    sim_logger.plot_gap_vs_time()
    # sim_logger.plot_relative_speeds('time', 'speed', r'\textbf{Relative Speed ($\Delta v$) Between Successive Vehicles Over Time}', start_time=0, end_time=2600)
    sim_logger.plot_time_space_diagram(start_time=0, end_time=2600)
    sim_logger.plot_spatiotemporal_heatmap(new_cmap=plt.cm.RdYlGn)
    # sim_logger.plot_spatiotemporal_diagram(start_time=0,end_time=500)