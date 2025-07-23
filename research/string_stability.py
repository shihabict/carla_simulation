import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq


custom_colors = [
            '#1f77b4',
            '#ff7f0e',
            '#2ca02c',
            '#d62728',
            '#9467bd',
            '#8c564b',
            '#e377c2',
            '#bcbd22',
        ]

def compute_transfer_ratio(leader_speed, follower_speed, dt):
    N = len(leader_speed)
    freq = fftfreq(N, d=dt)

    # FFT of leader and follower speeds
    L_fft = fft(leader_speed)
    F_fft = fft(follower_speed)

    # Transfer function magnitude (avoid div-by-zero)
    eps = 1e-6
    # magnitude_ratio = np.abs(F_fft) / (np.abs(L_fft) + eps)
    magnitude_ratio = np.abs(F_fft) / abs(L_fft)

    return freq[:N // 2], magnitude_ratio[:N // 2]  # return only positive freqs


def analyze_string_stability(csv_path, target_col, dt=0.02):
    df = pd.read_csv(csv_path)
    df = df.iloc[10000:25000]
    df = df[['time', 'name', target_col]]

    # Get unique followers (excluding leader)
    follower_names = sorted([n for n in df['name'].unique() if n != 'leader'])

    # Pivot dataframe for fast access
    pivot_df = df.pivot(index='time', columns='name', values=target_col)

    # Interpolate missing values
    # pivot_df = pivot_df.interpolate().dropna()
    leader_speed = pivot_df['leader'].values

    # Plot transfer magnitude for each follower
    plt.figure(figsize=(12, 6))
    for idx, follower in enumerate(follower_names):
        color = custom_colors[idx % len(custom_colors)]
        follower_speed = pivot_df[follower].values
        freq, ratio = compute_transfer_ratio(leader_speed, follower_speed, dt)

        plt.plot(freq, ratio, label=f"Δv: follower{idx+1}", color=color)
        leader_speed = follower_speed

    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude Ratio |V_f(jω)| / |V_l(jω)|')
    plt.title(f'String Stability via Transfer Function ({target_col})')
    plt.grid(True)
    plt.legend()
    plt.xlim(0, 1)  # Focus on low-frequency range
    plt.ylim(0, 2)
    plt.axhline(y=1.0, color='r', linestyle='--', label='Unity Gain')
    plt.tight_layout()
    # plt.show()
    plt.savefig(f"Reports/transfer_function_stability_{target_col}.png")
    # plt.savefig(
    #     f'Reports/acc_vs_time_{self.controller_type}_nV_{self.num_vehicle}_ref{self.reference_speed}_f{self.sampling_frequency}.png')


def plot_delta_v(csv_path: str, leader_name='leader'):
    df = pd.read_csv(csv_path)
    df = df.iloc[10000:25000]

    # Extract leader data
    leader_df = df[df['name'] == leader_name][['time', 'speed']].reset_index(drop=True)

    # Prepare the plot
    plt.figure(figsize=(12, 6))

    # Plot Δv for each follower
    for follower_name in df['name'].unique():
        if follower_name == leader_name:
            continue  # Skip leader

        follower_df = df[df['name'] == follower_name][['time', 'speed']].reset_index(drop=True)

        # Ensure equal length for subtraction
        min_len = min(len(leader_df), len(follower_df))
        delta_v = leader_df['speed'].iloc[:min_len] - follower_df['speed'].iloc[:min_len]
        time = follower_df['time'].iloc[:min_len]

        plt.plot(time, delta_v, label=f'Δv: {follower_name}')
        leader_df = follower_df

    plt.xlabel('Time (s)')
    plt.ylabel('Δv (m/s)')
    plt.title('Relative Speed Δv(t) between Leader and Followers')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    # plt.show()
    plt.savefig(f"Reports/relative_speed_stability.png")

def plot_follower_internal_delta_v(csv_path):
    df = pd.read_csv(csv_path)
    df = df.iloc[10000:25000]
    df = df[['time', 'name', 'speed']]

    # Get unique followers (excluding leader)
    follower_names = sorted([n for n in df['name'].unique() if n != 'leader'])

    # Pivot dataframe: time as index, names as columns, speed as values
    pivot_df = df.pivot(index='time', columns='name', values='speed')
    pivot_df = pivot_df.interpolate().dropna()

    # -----------------------------
    # Plot Δv(t) for each follower
    # -----------------------------
    plt.figure(figsize=(12, 6))
    for idx, follower in enumerate(follower_names):
        follower_speed = pivot_df[follower].values
        delta_v = np.diff(follower_speed)  # Δv = v(t) - v(t-1)
        time = pivot_df.index[1:]          # Align with Δv shape
        plt.plot(time, delta_v, label=f"{follower}", color=f"C{idx}")

    plt.xlabel("Time (s)")
    plt.ylabel("Δv(t) (m/s)")
    plt.title("Δv(t) of Each Follower (Speed Difference Over Time)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    # plt.show()
    plt.savefig("Reports/delta_v_self_follower_plot.png")

def compute_head_to_tail_amplification(csv_path, veq=None):
    """
    Compute Head-to-Tail Amplification metric A:
    A = max(|v_last(t) - veq|) / max(|v_leader(t) - veq|)

    Parameters:
        csv_path (str): Path to the CSV log file
        veq (float or None): Equilibrium speed (if None, use average leader speed)

    Returns:
        float: Amplification ratio A
    """
    # Load CSV
    df = pd.read_csv(csv_path)
    df = df.iloc[10000:25000]

    # Filter relevant columns
    df = df[['time', 'name', 'speed']]

    # Pivot for easy access
    pivot_df = df.pivot(index='time', columns='name', values='speed').dropna()

    # Get leader and last follower
    leader_speed = pivot_df['leader'].values
    follower_names = sorted([col for col in pivot_df.columns if col != 'leader'])
    last_follower_speed = pivot_df[follower_names[-1]].values

    # Determine equilibrium speed if not provided
    if veq is None:
        veq = np.mean(leader_speed)

    # Compute deviations
    leader_dev = np.abs(leader_speed - veq)
    last_follower_dev = np.abs(last_follower_speed - veq)

    # Compute amplification
    amplification = np.max(last_follower_dev) / np.max(leader_dev)

    return amplification

# Example usage
if __name__ == '__main__':
    # analyze_string_stability('Reports/sim_data_FS_IDM_nomi_nV_3_ref25_f0.02.csv',target_col='speed', dt=0.02)
    # analyze_string_stability('Reports/sim_data_FS_IDM_nomi_nV_3_ref25_f0.02.csv',target_col='acc', dt=0.02)
    # plot_delta_v('Reports/sim_data_FS_IDM_nomi_nV_3_ref25_f0.02.csv')
    # plot_follower_internal_delta_v('Reports/sim_data_FS_IDM_nomi_nV_3_ref25_f0.02.csv')
    ht_amplification = compute_head_to_tail_amplification('Reports/sim_data_FS_IDM_nomi_nV_3_ref25_f0.02.csv')
    print(ht_amplification)
