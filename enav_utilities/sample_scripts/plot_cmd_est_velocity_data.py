#!/usr/bin/env python

""" plot_cmd_est_velocity_data.py

    Plot commanded and estimated velocities of the enav dataset

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
    cmd_vel_data = bag_obj.load_cmd_vel_data(rel_time=True)
    est_vel_data = bag_obj.load_est_vel_data(rel_time=True)

    # Plot data
    plt.figure(1)
    ax1 = plt.subplot(221)
    ax1.plot(cmd_vel_data[:,0], cmd_vel_data[:,1], label="x axis")
    ax1.plot(cmd_vel_data[:,0], cmd_vel_data[:,2], label="y axis")
    ax1.plot(cmd_vel_data[:,0], cmd_vel_data[:,3], label="z axis")
    ax1.set_ylabel('Linear velocities [m/s]')
    ax1.set_title("Commanded linear and angular velocities \nof the rover base vs time")
    ax1.legend(loc='upper right')

    ax2 = plt.subplot(223)
    ax2.plot(cmd_vel_data[:,0], cmd_vel_data[:,4], label="x axis")
    ax2.plot(cmd_vel_data[:,0], cmd_vel_data[:,5], label="y axis")
    ax2.plot(cmd_vel_data[:,0], cmd_vel_data[:,6], label="z axis")
    ax2.set_ylabel('Angular velocities [rad/s]')
    ax2.set_xlabel('Time [s]')
    ax2.legend(loc='upper right')

    ax3 = plt.subplot(222)
    ax3.plot(est_vel_data[:,0], est_vel_data[:,1], label="x axis")
    ax3.plot(est_vel_data[:,0], est_vel_data[:,2], label="y axis")
    ax3.plot(est_vel_data[:,0], est_vel_data[:,3], label="z axis")
    ax3.set_ylabel('Linear velocities [m/s]')
    ax3.set_title("Estimated linear and angular velocities \nof the rover base vs time")
    ax3.legend(loc='upper right')

    ax4 = plt.subplot(224)
    ax4.plot(est_vel_data[:,0], est_vel_data[:,4], label="x axis")
    ax4.plot(est_vel_data[:,0], est_vel_data[:,5], label="y axis")
    ax4.plot(est_vel_data[:,0], est_vel_data[:,6], label="z axis")
    ax4.set_ylabel('Angular velocities [rad/s]')
    ax4.set_xlabel('Time [s]')
    ax4.legend(loc='upper right')

    plt.show()
