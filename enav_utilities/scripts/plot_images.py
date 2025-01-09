#!/usr/bin/env python

""" plot_images.py

    Plot images of the ENAV dataset

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
    Affl.:  Space and Terrestrial Autonomous Robotic Systems Laboratory
            University of Toronto
    Date:   February 10, 2019
    """


import argparse
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

from enav_utilities.rosbag_loader import ENAVRosbagLoader


# Parse bag file name
parser = argparse.ArgumentParser()
parser.add_argument("-b", "--bag", help="Path to rosbag file")
args = parser.parse_args()


if __name__ == "__main__":

    # Fetch images between 60 and 62 seconds after the rosbag start time
    bag_obj = ENAVRosbagLoader(args.bag)

    mono_times, mono_img_data = bag_obj.load_image_data(
        img_source="mono_image", time_range=(60, 62), rel_time=True)
    omni2_times, omni2_img_data = bag_obj.load_image_data(
        img_source="omni_image2", time_range=(60, 62), rel_time=True)
    omni4_times, omni4_img_data = bag_obj.load_image_data(
        img_source="omni_image4", time_range=(60, 62), rel_time=True)
    pan_times, pan_img_data = bag_obj.load_image_data(
        img_source="omni_stitched_image", time_range=(60, 62), rel_time=True)
    pandisp_times, pandisp_img_data = bag_obj.load_image_data(
        img_source="omni_stitched_disparity", time_range=(60, 62), rel_time=True)

    # Plot first image of requested time range (60 seconds after bag start)
    fig = plt.figure(dpi=100)
    gs = gridspec.GridSpec(3,3)

    ax = fig.add_subplot(gs[0,0])
    ax.imshow(mono_img_data[0,...], cmap='gray')
    ax.set_title('Monocular camera raw image')
    plt.axis('off')

    ax = fig.add_subplot(gs[0,1])
    ax.imshow(omni2_img_data[0,...])
    ax.set_title('Omni2 camera raw image')
    plt.axis('off')

    ax = fig.add_subplot(gs[0,2])
    ax.imshow(omni4_img_data[0,...])
    ax.set_title('Omni4 camera raw image')
    plt.axis('off')

    ax = fig.add_subplot(gs[1,:])
    ax.imshow(pan_img_data[0,...])
    ax.set_title('Panoramic RGB image')
    plt.axis('off')

    ax = fig.add_subplot(gs[2,:])
    ax.imshow(pandisp_img_data[0,...])
    ax.set_title('Panoramic disparity image')
    plt.axis('off')

    plt.suptitle("Images 60 seconds from rosbag start time")
    plt.show()
