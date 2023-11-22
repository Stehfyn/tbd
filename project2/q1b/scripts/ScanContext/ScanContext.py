import numpy
from datetime import datetime
import matplotlib.pyplot as plt

class ScanContext:
    def __init__(self, duration_in_s = 5):
        self.data = numpy.array(None)
        self.scan_start = datetime.now()
        self.scan_duration = duration_in_s

    
    def add_laser_scan(self, ls):
        angles = numpy.array([ls.angle_min + (i * ls.angle_increment) for i in range(0, 12)], dtype=float)
        ranges = numpy.array(ls.ranges, dtype=float)
        intensities = numpy.array(ls.intensities, dtype=float)
        data = numpy.column_stack((angles,ranges,intensities))

        if not (self.data.ndim and self.data.size):
            self.data = data
        else:
            self.data = numpy.vstack((self.data, data))

    def still_scanning(self) -> bool:
        return int((datetime.now() - self.scan_start).total_seconds()) < self.scan_duration
    
    def serialize(self):
        if (self.data.ndim and self.data.size):
            numpy.savetxt(f"LidarScanContext.txt", self.data)

    def deserialize(self, path):
        self.data = numpy.loadtxt(path, dtype=float)

    def plot_point_cloud(self):
        if (self.data.ndim and self.data.size):
            angles = self.data[:, 0]
            ranges = self.data[:, 1]

            # Convert polar coordinates to Cartesian coordinates
            x = ranges * numpy.cos(angles)
            y = ranges * numpy.sin(angles)

            # Create subplots
            fig, axs = plt.subplots(1, 2, figsize=(12, 6))

            # Polar plot
            axs[0] = plt.subplot(1, 2, 1, polar=True)
            axs[0].scatter(angles, ranges)
            axs[0].set_xlabel('Angles (radians)')
            axs[0].set_ylabel('Range')
            axs[0].set_title('Polar Coordinates Plot')

            # Cartesian plot
            axs[1].scatter(x, y)
            axs[1].set_xlabel('X Coordinate')
            axs[1].set_ylabel('Y Coordinate')
            axs[1].set_title('Cartesian Coordinates Plot')

            plt.tight_layout()
            plt.show()

    def get_data(self):
        return self.data