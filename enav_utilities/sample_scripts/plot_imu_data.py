#!/usr/bin/env python

""" plot_imu_data.py

    Plot IMU data of the enav dataset

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
    imu_data = bag_obj.load_imu_data(rel_time=True)

    # Plot data
    plt.figure(1)
    ax1 = plt.subplot(411)
    plt.plot(imu_data[:,0], imu_data[:,1], label="x axis")
    plt.plot(imu_data[:,0], imu_data[:,2], label="y axis")
    plt.plot(imu_data[:,0], imu_data[:,3], label="z axis")
    plt.ylabel('Linear accelerations [m/s^2]')
    plt.title("IMU linear accelerations, angular velocities, \n and orientations vs time")
    plt.legend(loc='upper right')

    plt.subplot(412, sharex=ax1)
    plt.plot(imu_data[:,0], imu_data[:,4], label="x axis")
    plt.plot(imu_data[:,0], imu_data[:,5], label="y axis")
    plt.plot(imu_data[:,0], imu_data[:,6], label="z axis")
    plt.ylabel('Angular velocities [rad/s]')
    plt.legend(loc='upper right')

    plt.subplot(413, sharex=ax1)
    plt.plot(imu_data[:,0], imu_data[:,7], label="q_x")
    plt.plot(imu_data[:,0], imu_data[:,8], label="q_y")
    plt.plot(imu_data[:,0], imu_data[:,9], label="q_z")
    plt.plot(imu_data[:,0], imu_data[:,10], label="q_w")
    plt.ylabel('Orientations [quaternion]')
    plt.xlabel('Time [s]')
    plt.legend(loc='upper right')

    plt.show()
