#!/usr/bin/env python

""" plot_rasters.py

    Plot georeferenced maps of the test environment

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
    Affl.:  Space and Terrestrial Autonomous Robotic Systems Laboratory
            University of Toronto
    Date:   February 10, 2019
    """

import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1 import make_axes_locatable

from enav_utilities.maps_loader import ENAVMapsLoader


parser = argparse.ArgumentParser()
parser.add_argument("-d", "--directory", help="Path to maps directory")
args = parser.parse_args()


if __name__=="__main__":

    site = ENAVMapsLoader(args.directory)

    # Show site info
    print("Width [m]: {0:.2f}".format(site.width))
    print("Height [m]: {0:.2f}".format(site.height))
    print("Easting spatial resolution [m/px]: {0:.3f}".format(site.res_easting))
    print("Northing spatial resolution [m/px]: {0:.3f}".format(site.res_northing))
    print("Map extent in UTM coords (left, right, bottom, top): {}".format(site.extent_utm))

    # Plot all rasters
    fig, axes = plt.subplots(2, 2, dpi=100)# figsize=(15,10))

    im00 = axes[0,0].imshow(site.mosaic_rgb, extent=site.extent_utm)
    axes[0,0].set_title("RGB mosaic")

    im01 = axes[0,1].imshow(site.dem, extent=site.extent_utm, cmap='gray')
    axes[0,1].set_title("Elevation above origin (NW corner) [meters]")
    axes[0,1].scatter(*site.origin_utm, c='r', s=50, label="Terrain origin")
    axes[0,1].legend(scatterpoints = 1)

    im10 = axes[1,0].imshow(site.slope, extent=site.extent_utm, cmap='jet')
    axes[1,0].set_title("Slope map [radians]")

    im11 = axes[1,1].imshow(site.aspect, extent=site.extent_utm, cmap='jet')
    axes[1,1].set_title("Aspect map [radians]")

    for elem in np.nditer(axes, flags=['refs_ok']):
        elem.item().set_xlabel('Easting [m]')
        elem.item().set_ylabel('Northing [m]')

    plt.suptitle("Georeferenced maps in UTM coordinates (UTM zone {})".format(site.zone_utm))

    # Colorbars
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
