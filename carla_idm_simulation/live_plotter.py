

import matplotlib.pyplot as plt

class LivePlotter:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Leader vs Follower Speed')

        self.timestamps = []
        self.leader_speeds = []
        self.follower_speeds = []

        self.leader_line, = self.ax.plot([], [], 'b-', label='Leader Speed')
        self.follower_line, = self.ax.plot([], [], 'r-', label='Follower Speed')

        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.set_title("Live Speed Plot")
        self.ax.grid(True)
        self.ax.legend()

        self.fig.show()

    def update(self, t, leader_v, follower_v):
        self.timestamps.append(t)
        self.leader_speeds.append(leader_v)
        self.follower_speeds.append(follower_v)

        self.leader_line.set_data(self.timestamps, self.leader_speeds)
        self.follower_line.set_data(self.timestamps, self.follower_speeds)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)  # ⬅️ This is critical for live update
