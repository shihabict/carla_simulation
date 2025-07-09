import pandas as pd
import numpy as np

class SpeedProfileController:
    def __init__(self, csv_path: str):
        self.df = pd.read_csv(csv_path)
        self._prepare_profile()

    def _prepare_profile(self):
        self.df['speed_mps'] = self.df['Message'] * (1000 / 3600)  # km/h to m/s
        # Drop initial rows where speed is 0
        first_non_zero_idx = self.df[self.df['speed_mps'] > 0].index[0]
        self.df = self.df.iloc[first_non_zero_idx:].reset_index(drop=True)

        # Normalize time to start from zero
        self.df['time_rel'] = self.df['Time'] - self.df['Time'].iloc[0]
        self.df['time_diff'] = self.df['time_rel'].diff().fillna(0)

    def get_speed_at(self, sim_time: float) -> float:
        return np.interp(sim_time, self.df['time_rel'], self.df['speed_mps'])