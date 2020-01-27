#!/usr/bin/env python

""" show_plot.py

    Plot raster data of the CSA MET for demonstration/visualization purposes

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>

    Affl.:  Space and Terrestrial Autonomous Robotic Systems Laboratory
            University of Toronto

    Date:   February 10, 2019
    """

import sys

import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

sys.path.append("..")
from csa_raster_load import CSASite


# Parse directory name
parser = argparse.ArgumentParser()
parser.add_argument("-d", "--directory",
                    help="Path to CSA raster data directory")
args = parser.parse_args()


if __name__=="__main__":
    site = CSASite(args.directory)
    site_width = site.extent_utm[1] - site.extent_utm[0]
    site_height = site.extent_utm[3] - site.extent_utm[2]

    # Show site info
    print("Site width [m]: ", site_width)
    print("Site height [m]: ", site_height)
    print("Site mosaic res y [m/px]: ", site_height/site.mosaic_rgb.shape[0])
    print("Site mosaic res x [m/px]: ", site_width/site.mosaic_rgb.shape[1])
    print("Site extent: ", site.extent_utm)

    # Plot all maps
    fig, axes = plt.subplots(2, 2, figsize=(15,10))

    im00 = axes[0,0].imshow(site.mosaic_rgb, extent=site.extent_utm)
    axes[0,0].set_title("RGB mosaic")
    axes[0,0].set_xlabel('Easting [m]')
    axes[0,0].set_ylabel('Northing [m]')

    im01 = axes[0,1].imshow(site.dem, extent=site.extent_utm, cmap='gray')
    axes[0,1].set_title("Elevation above origin (NW corner) [meters]")
    axes[0,1].set_xlabel('Easting [m]')
    axes[0,1].set_ylabel('Northing [m]')

    im10 = axes[1,0].imshow(site.slope, extent=site.extent_utm, cmap='jet')
    axes[1,0].set_title("Slope map [radians]")
    axes[1,0].set_xlabel('Easting [m]')
    axes[1,0].set_ylabel('Northing [m]')

    im11 = axes[1,1].imshow(site.aspect, extent=site.extent_utm, cmap='jet')
    axes[1,1].set_title("Aspect map [radians]")
    axes[1,1].set_xlabel('Easting [m]')
    axes[1,1].set_ylabel('Northing [m]')

    plt.suptitle("All georeferenced maps in UTM coordinates (UTM zone {})".format(site.zone_utm))

    # Show colorbars
    div01 = make_axes_locatable(axes[0,1])
    cax01 = div01.append_axes("right", size="5%", pad=0.1)
    plt.colorbar(im01, cax=cax01)

    div10 = make_axes_locatable(axes[1,0])
    cax10 = div10.append_axes("right", size="5%", pad=0.1)
    plt.colorbar(im10, cax=cax10)

    div11 = make_axes_locatable(axes[1,1])
    cax11 = div11.append_axes("right", size="5%", pad=0.1)
    plt.colorbar(im11, cax=cax11)

    plt.show()
