#!/usr/bin/env python

""" csa_raster_load.py

    Load raster data of the CSA MET

    Author:     Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>

    Affl.:      Space and Terrestrial Autonomous Robotic Systems Laboratory
                University of Toronto
    Date:       February 10, 2019
    """

import os
import utm
import numpy as np
import rasterio
import rasterio.plot


class CSASite():

    _CSA_ORIGIN_LATLON = (45.518206644445, -73.393904468182)
    _CSA_ORIGIN_UTM = utm.from_latlon(*_CSA_ORIGIN_LATLON)[0:2]
    _CSA_THETA_NORTH_RAD = np.radians(0.5)

    def __init__(self, dirname):
        self.dirname = dirname

        # Load site settings
        try:
            self.load_data_from_directory(self.dirname)
        except Exception as e:
            raise IOError("Could not load maps from {}: {}".format(self.dirname, e))

    def load_data_from_directory(self, dirname):

        # GeoTIFF maps
        self.gtif = {
            'mosaic': rasterio.open(os.path.join(dirname, "mosaic_utm_20cm.tif")),
            'dem': rasterio.open(os.path.join(dirname, 'dem_utm_20cm.tif')),
            'slope': rasterio.open(os.path.join(dirname, 'slopemod_utm_20cm.tif')),
            'aspect': rasterio.open(os.path.join(dirname, 'aspect_utm_20cm.tif')),
            }
        
        for datatype in self.gtif:
            setattr(self, datatype, self.get_raster(datatype))
        
        # Generate rgb mosaic
        mosaic_rgb = np.dstack(tuple(self.gtif['mosaic'].read(i) for i in [1,2,3]))
        setattr(self, 'mosaic_rgb', mosaic_rgb)

        # Adjustment of slope & aspect
        self.slope[self.slope < 0] = 0
        self.aspect[self.aspect < 0] = 0
        self.slope = np.deg2rad(self.slope)
        self.aspect = np.deg2rad(self.aspect)
    
    def get_raster(self, key):
        return self.gtif[key].read(1)
    
    @property
    def extent_utm(self):
        """Return Matplotlib plotting extent in UTM coords"""

        return rasterio.plot.plotting_extent(self.gtif['mosaic'])

    @property
    def zone_utm(self):
        """Return the UTM zone code of the CSA MET"""
        return "18T"


if __name__=="__main__":
    pass