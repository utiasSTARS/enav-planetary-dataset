#!/usr/bin/env python

""" plot_energy.py

    Plot power & energy consumption of the enav dataset

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>

    Affl.:  Space and Terrestrial Autonomous Robotic Systems Laboratory
            University of Toronto

    Date:   February 10, 2019
    """

import sys
import argparse
import matplotlib.pyplot as plt

sys.path.append("..")
from rosbag_data_load import FetchEnergyDataset


# Parse bag file name
parser = argparse.ArgumentParser()
parser.add_argument("-b", "--bag_file", help="path to rosbag file")
args = parser.parse_args()


if __name__ == "__main__":

    # Fetch data
    bag_obj = FetchEnergyDataset(args.bag_file)
    power_data = bag_obj.load_energy_data(rel_time=True)

    # Plot data
    plt.figure(1)
    ax1 = plt.subplot(411)
    plt.plot(power_data[:,0], power_data[:,1], label="Left driver")
    plt.plot(power_data[:,0], power_data[:,3], label="Right driver")
    plt.ylabel('Voltage (V)')
    plt.title("Left and right motor drivers voltages, currents, \n power and total energy spent vs time")
    plt.legend(loc='upper right')

    plt.subplot(412, sharex=ax1)
    plt.plot(power_data[:,0], power_data[:,2], label="Left driver")
    plt.plot(power_data[:,0], power_data[:,4], label="Right driver")
    plt.ylabel('Current (A)')
    plt.legend(loc='upper right')

    plt.subplot(413, sharex=ax1)
    plt.plot(power_data[:,0], power_data[:,5], label="Left driver")
    plt.plot(power_data[:,0], power_data[:,6], label="Right driver")
    plt.ylabel('Power (W)')
    plt.legend(loc='upper right')

    plt.subplot(414, sharex=ax1)
    plt.plot(power_data[:,0], power_data[:,7]/3600, label="Left driver")
    plt.plot(power_data[:,0], power_data[:,8]/3600, label="Right driver")
    plt.xlabel('Time (s)')
    plt.ylabel('Energy (Wh)')
    plt.legend(loc='upper right')

    plt.show()
