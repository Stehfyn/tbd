import numpy
from datetime import datetime
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

class ScanContext:
    def __init__(self, duration_in_s = 5):
        self.data = numpy.array(None)
        self.scan_start = datetime.now()
        self.scan_duration = duration_in_s
        self.range_mask = 4000

    def set_range_mask(self, range_mask):
        self.range_mask = range_mask

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

    def plot_point_cloud(self, write_to_disk=False):
        if (self.data.ndim and self.data.size):
            angles = self.data[:, 0]
            ranges = self.data[:, 1]

            mask = ranges <= self.range_mask
            angles = angles[mask]
            ranges = ranges[mask]

            ranges *= .001

            # Convert polar coordinates to Cartesian coordinates
            x = ranges * numpy.cos(angles)
            y = ranges * numpy.sin(angles)

            # Combine angles and ranges into a single array for clustering
            X = numpy.column_stack((x, y))

            # Perform DBSCAN clustering
            # eps and min_samples can be adjusted based on your data
            db = DBSCAN(eps=.2, min_samples=60).fit(X)
            labels = db.labels_

            

            # Create subplots
            fig, axs = plt.subplots(1, 2, figsize=(12, 6))

            # Polar plot
            axs[0] = plt.subplot(1, 2, 1, polar=True)
            scatter = axs[0].scatter(angles, ranges, c=labels, cmap='viridis')
            axs[0].scatter(angles, ranges)
            axs[0].set_xlabel('Angles (radians)')
            axs[0].set_ylabel('Range')
            axs[0].set_title('Polar Coordinates Plot')

            # Cartesian plot
            #axs[1].scatter(x, y)
            axs[1].scatter(x, y, c=labels, cmap='viridis')
            axs[1].set_xlabel('X Coordinate')
            axs[1].set_ylabel('Y Coordinate')
            axs[1].set_title('Cartesian Coordinates Plot')

            plt.tight_layout()

            plt.savefig('lidar_scan_plot.png') if write_to_disk else plt.show()
            print(len(labels))

    def get_data(self):
        return self.data

