import matplotlib.pyplot as plt

class LivePlotter:

    def __init__(self, vehicle_labels):
        import matplotlib.pyplot as plt
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Leader & Followers Speed')

        self.vehicle_labels = vehicle_labels
        self.timestamps = []
        self.speeds = {label: [] for label in vehicle_labels}
        self.lines = {}

        colors = ['blue', 'orange', 'green', 'red', 'purple', 'brown', 'cyan']
        for i, label in enumerate(vehicle_labels):
            line, = self.ax.plot([], [], label=label, color=colors[i % len(colors)])
            self.lines[label] = line

        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.set_title("Live Speed Plot")
        self.ax.legend()
        self.ax.grid(True)

        self.fig.show()

    def update(self, t, vehicle_speeds_dict):
        self.timestamps.append(t)
        for label, speed in vehicle_speeds_dict.items():
            self.speeds[label].append(speed)
            self.lines[label].set_xdata(self.timestamps)
            self.lines[label].set_ydata(self.speeds[label])

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        plt.pause(0.001)

    def save_plot(self, filename='speed_plot.png'):
        """
        Save the current plot to a file.
        Parameters:
        - filename (str): Name of the output file (e.g., 'plot.png', 'plot.pdf')
        """
        self.ax.legend()  # Ensure legend is drawn in saved version
        self.fig.savefig(filename, bbox_inches='tight')
        print(f"Plot saved to '{filename}'")



