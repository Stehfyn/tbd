import numpy
from datetime import datetime
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import pandas as pd

class ScanContext:
    def __init__(self, duration_in_s = 5):
        self.data = numpy.array(None)
        self.data_df = None
        self.scan_start = datetime.now()
        self.scan_duration = duration_in_s
        self.range_mask = 4

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
    
    def serialize(self, parent):
        if (self.data.ndim and self.data.size):
            numpy.savetxt(f"{parent}/LidarScanContext.txt", self.data)

    def serialize_as_df(self, parent):
        if (self.data.ndim and self.data.size):
            dataframe = self.__to_dataframe__()
            dataframe.to_csv(f"{parent}/LidarScanContext.csv")
    
    def __to_dataframe__(self):
        if (self.data.ndim and self.data.size):
            #return pd.DataFrame({'angles': self.data[:, 0], 'ranges': self.data[:, 1], 'intensity': self.data[:, 2]})
            angles = self.data[:, 0]
            ranges = self.data[:, 1]
            mask = ranges <= 2.5
            angles = angles[mask]
            ranges = ranges[mask]

            return pd.DataFrame({'angles': angles, 'ranges': ranges})

    def deserialize(self, path):
        self.data = numpy.loadtxt(path, dtype=float)
        self.data_df = self.__to_dataframe__()

    def plot_point_cloud(self, write_to_disk=False, path=''):
        if (self.data.ndim and self.data.size):
            angles = self.data[:, 0]
            ranges = self.data[:, 1]

            mask = ranges <= 2.5
            angles = angles[mask]
            ranges = ranges[mask]

            # Convert polar coordinates to Cartesian coordinates
            x = ranges * numpy.cos(angles)
            y = ranges * numpy.sin(angles)

            # Create subplots
            fig, axs = plt.subplots(1, 2, figsize=(12, 6))

            # Polar plot
            axs[0] = plt.subplot(1, 2, 1, polar=True)
            scatter = axs[0].scatter(angles, ranges)
            axs[0].scatter(angles, ranges)
            axs[0].set_xlabel('Angles (radians)')
            axs[0].set_ylabel('Range')
            axs[0].set_title('Polar Coordinates Plot')

            # Cartesian plot
            #axs[1].scatter(x, y)
            axs[1].scatter(x, y)
            axs[1].set_xlabel('X Coordinate')
            axs[1].set_ylabel('Y Coordinate')
            axs[1].set_title('Cartesian Coordinates Plot')

            plt.tight_layout()

            plt.savefig(f'{path}/lidar_scan_plot.png') if write_to_disk else plt.show()

    def get_data(self):
        return self.data

    def get_data_df(self):
        return self.data_df

