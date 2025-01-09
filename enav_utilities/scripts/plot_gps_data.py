#!/usr/bin/env python

""" plot_gps_data.py

    Spatially plot GPS data of the enav dataset

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
    Affl.:  Space and Terrestrial Autonomous Robotic Systems Laboratory
            University of Toronto
    Date:   February 10, 2019
    """

import argparse
import matplotlib.pyplot as plt

from enav_utilities.rosbag_loader import ENAVRosbagLoader
from enav_utilities.maps_loader import ENAVMapsLoader


# Parse bag file name
parser = argparse.ArgumentParser()
parser.add_argument("-b", "--bag_file", help="path to rosbag file")
parser.add_argument("-d", "--directory", help="path to CSA raster data directory")
args = parser.parse_args()

if __name__ == "__main__":

    # Fetch data
    bag_obj = ENAVRosbagLoader(args.bag_file)
    gps_data = bag_obj.load_gps_data(ret_utm=True, rel_time=True)
    site = ENAVMapsLoader(args.directory)

    # Plot data
    fig = plt.figure(dpi=100)
    plt.imshow(site.mosaic_rgb, extent=site.extent_utm)
    plt.scatter(gps_data[:,1], gps_data[:,2], s=1, color='b')

    # Start & end points
    plt.scatter(
        gps_data[0,1], gps_data[0,2], s=250, facecolors='none',
        marker='o', linewidths=3, edgecolors='b', label='Start'
    )
    plt.scatter(
        gps_data[-1,1], gps_data[-1,2], s=250, facecolors='b',
        marker='o', edgecolors='b', label='End'
    )

    plt.xlabel('Easting [m]', fontsize=13)
    plt.ylabel('Northing [m]', fontsize=13)
    plt.legend(scatterpoints = 1)

    plt.show()
