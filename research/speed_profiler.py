import pandas as pd
import numpy as np

from settings import ROOT_DIR


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
        indices = np.arange(0, len(self.df), 2)
        # self.df['speed_mps'] = self.df['speed_mps'].round(3)
        self.df = self.df.iloc[indices].reset_index(drop=True)
        self.df = self.df.round(3)


    def get_speed_at(self, sim_time: float) -> float:
        return np.interp(sim_time, self.df['time_rel'], self.df['speed_mps'])

    def downsample_df(self,step):
        indices = np.arange(0, len(self.df), step)
        return self.df.iloc[indices].reset_index(drop=True)



if __name__ == '__main__':
    csv_path = f'{ROOT_DIR}/datasets/CAN_Messages_decoded_speed.csv'
    speed_profiler = SpeedProfileController(csv_path)
    speed_profiler.save_processed_data()