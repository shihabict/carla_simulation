import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq


custom_colors = [
            '#1f77b4',  # blue
            '#ff7f0e',  # orange
            '#2ca02c',  # green
            '#d62728',  # red
            '#9467bd',  # purple
            '#8c564b',  # brown
            '#e377c2',  # pink
            '#bcbd22',  # yellow-green
        ]

def compute_transfer_ratio(leader_speed, follower_speed, dt):
    """
    Compute transfer function magnitude (|V_f(jω)| / |V_l(jω)|) using FFT.
    """
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


def analyze_string_stability(csv_path, dt=0.02):
    df = pd.read_csv(csv_path)
    df = df.iloc[20000:50000]
    df = df[['time', 'name', 'speed']]

    # Get unique followers (excluding leader)
    follower_names = sorted([n for n in df['name'].unique() if n != 'leader'])

    # Pivot dataframe for fast access
    pivot_df = df.pivot(index='time', columns='name', values='speed')

    # Interpolate missing values
    # pivot_df = pivot_df.interpolate().dropna()

    leader_speed = pivot_df['leader'].values

    # Plot transfer magnitude for each follower
    plt.figure(figsize=(12, 6))
    for idx, follower in enumerate(follower_names):
        color = custom_colors[idx % len(custom_colors)]
        follower_speed = pivot_df[follower].values
        freq, ratio = compute_transfer_ratio(leader_speed, follower_speed, dt)

        plt.plot(freq, ratio, label=f"follower{idx+1}", color=color)
        leader_speed = follower_speed

    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude Ratio |V_f(jω)| / |V_l(jω)|')
    plt.title('String Stability via Transfer Function')
    plt.grid(True)
    plt.legend()
    plt.xlim(0, 1)  # Focus on low-frequency range
    plt.ylim(0, 2)
    plt.axhline(y=1.0, color='r', linestyle='--', label='Unity Gain')
    plt.tight_layout()
    # plt.show()
    plt.savefig(f"Reports/transfer_function_stability.png")
    # plt.savefig(
    #     f'Reports/acc_vs_time_{self.controller_type}_nV_{self.num_vehicle}_ref{self.reference_speed}_f{self.sampling_frequency}.png')

# Example usage
if __name__ == '__main__':
    analyze_string_stability('Reports/sim_data_FS_IDM_nomi_nV_3_ref25_f0.02.csv', dt=0.02)
