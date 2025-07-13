import pandas as pd
import numpy as np

from settings import ROOT_DIR


# class SpeedProfileController:
#     def __init__(self, csv_path: str):
#         self.df = pd.read_csv(csv_path)
#         self._prepare_profile()
#
#     def _prepare_profile(self):
#         self.df['speed_mps'] = self.df['Message'] * (1000 / 3600)  # km/h to m/s
#         # Drop initial rows where speed is 0
#         first_non_zero_idx = self.df[self.df['speed_mps'] > 0].index[0]
#         self.df = self.df.iloc[first_non_zero_idx:].reset_index(drop=True)
#
#         # Normalize time to start from zero
#         self.df['time_rel'] = self.df['Time'] - self.df['Time'].iloc[0]
#         self.df['time_diff'] = self.df['time_rel'].diff().fillna(0)
#         indices = np.arange(0, len(self.df), 2)
#         self.df = self.df.iloc[indices].reset_index(drop=True)
#         self.df = self.df.round(3)
#
#
#     def get_speed_at(self, sim_time: float) -> float:
#         return np.interp(sim_time, self.df['time_rel'], self.df['speed_mps'])
#
#     def downsample_df(self,step):
#         indices = np.arange(0, len(self.df), step)
#         return self.df.iloc[indices].reset_index(drop=True)

# import pandas as pd
# import numpy as np

class SpeedProfileController:
    def __init__(self, csv_path: str, target_rate: int = 100):
        self.target_rate = target_rate
        self.df = pd.read_csv(csv_path)
        self._prepare_profile()

    def _prepare_profile(self):
        self.df['speed_mps'] = self.df['Message'] * (1000 / 3600)  # Convert km/h to m/s

        # Drop initial zero-speed rows
        first_non_zero_idx = self.df[self.df['speed_mps'] > 0].index[0]
        self.df = self.df.iloc[first_non_zero_idx:].reset_index(drop=True)

        # Compute relative time and sampling interval
        self.df['time_rel'] = self.df['Time'] - self.df['Time'].iloc[0]
        self.df['time_diff'] = self.df['time_rel'].diff().fillna(0)

        # Estimate original sampling frequency
        avg_interval = self.df['time_diff'].mean()
        original_rate = 1 / avg_interval if avg_interval > 0 else 50

        # Compute downsampling step to match target_rate
        step = int(round(original_rate / self.target_rate))
        step = max(1, step)

        # Downsample the dataframe
        # self.df = self.df.iloc[::step].reset_index(drop=True)
        self.df = self.df.round(3)
        self.df = self.df[self.df['time_diff']!=0].reset_index(drop=True)
        self.downsample_to_10hz()
        print(0)



    def get_speed_at(self, sim_time: float) -> float:
        return np.interp(sim_time, self.df['time_rel'], self.df['speed_mps'])

    def downsample_df(self, step: int):
        indices = np.arange(0, len(self.df), step)
        return self.df.iloc[indices].reset_index(drop=True)

    def save_processed_data(self, path='processed_speed_profile.csv'):
        self.df.to_csv(path, index=False)
        print(f"[Saved] Downsampled speed profile saved to {path}")

    def downsample_to_10hz(self):
        # Group by each second
        self.df['second'] = self.df['time_rel'].astype(int)

        # Take 10 evenly spaced samples within each second
        downsampled = self.df.groupby('second', group_keys=False).apply(
            lambda g: g.iloc[np.linspace(0, len(g) - 1, min(10, len(g))).astype(int)])

        # Drop helper column and reset index
        downsampled = downsampled.drop(columns='second').reset_index(drop=True)
        self.df = downsampled


if __name__ == '__main__':
    csv_path = f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv'
    speed_profiler = SpeedProfileController(csv_path)
    speed_profiler.save_processed_data()