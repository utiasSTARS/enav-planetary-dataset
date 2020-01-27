#!/usr/bin/env python

""" plot_images.py

    Plot images of the enav dataset

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>

    Affl.:  Space and Terrestrial Autonomous Robotic Systems Laboratory
            University of Toronto

    Date:   February 10, 2019
    """

import sys

import argparse
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

sys.path.append("..")
from rosbag_data_load import FetchEnergyDataset


# Parse bag file name
parser = argparse.ArgumentParser()
parser.add_argument("-b", "--bag_file", help="path to rosbag file")
args = parser.parse_args()


if __name__ == "__main__":

    # Fetch data
    bag_obj = FetchEnergyDataset(args.bag_file)

    mono_times, mono_img_data = bag_obj.load_image_data(img_source="mono_image", time_range=(1536096176, 1536096177))
    omni2_times, omni2_img_data = bag_obj.load_image_data(img_source="omni_image2", time_range=(1536096176, 1536096177))
    omni4_times, omni4_img_data = bag_obj.load_image_data(img_source="omni_image4", time_range=(1536096176, 1536096177))
    pan_times, pan_img_data = bag_obj.load_image_data(img_source="omni_stitched_image", time_range=(1536096176, 1536096177))
    pandisp_times, pandisp_img_data = bag_obj.load_image_data(img_source="omni_stitched_disparity", time_range=(1536096176, 1536096177))
     
    # Plot data
    fig = plt.figure()
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

    plt.show()
