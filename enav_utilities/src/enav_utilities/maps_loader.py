#!/usr/bin/env python

""" raster_load.py

    Load rasterized maps of the test environment

    Author:     Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
    Affl.:      Space and Terrestrial Autonomous Robotic Systems Laboratory
                University of Toronto
    Date:       February 10, 2019
    """

import numpy as np
from pathlib import Path
import rasterio
from rasterio.plot import plotting_extent
import utm


class ENAVMapsLoader:

    # Map origin (North West corner or terrain) latitude & longitude (degrees)
    _ORIGIN_LATLON = (45.518206644445, -73.393904468182)

    _UTM_ZONE = "18T"

    def __init__(self, dirpath):
        """Init ENAV site & load maps

        Args:
            dirpath (Path): path to the maps directory
        """

        try:
            self.load_data_from_directory(dirpath)
        except Exception as e:
            raise IOError("Could not load maps from {}: {}".format(dirpath, e))

    def load_data_from_directory(self, dirpath):

        # GeoTIFF maps
        self.gtif = {
            'mosaic': rasterio.open(str(Path(dirpath, "mosaic_utm_20cm.tif"))),
            'dem': rasterio.open(str(Path(dirpath, 'dem_utm_20cm.tif'))),
            'slope': rasterio.open(str(Path(dirpath, 'slopemod_utm_20cm.tif'))),
            'aspect': rasterio.open(str(Path(dirpath, 'aspect_utm_20cm.tif'))),
            }

        for datatype in self.gtif:
            setattr(self, datatype, self.get_raster(datatype))

        # RGB mosaic
        mosaic_rgb = np.dstack(tuple(self.get_raster('mosaic', i) for i in range(1,4)))
        setattr(self, 'mosaic_rgb', mosaic_rgb)

        # Adjustment of slope & aspect
        self.slope[self.slope < 0] = 0
        self.aspect[self.aspect < 0] = 0
        self.slope = np.deg2rad(self.slope)
        self.aspect = np.deg2rad(self.aspect)

    def get_raster(self, key, band=1):
        return self.gtif[key].read(band)

    @property
    def extent_utm(self):
        """Matplotlib plotting extent in UTM coords (left, right, bottom, top)"""
        return plotting_extent(self.gtif['mosaic'])  # same for all rasters

    @property
    def width(self):
        """Map width (along easting direction) in meters"""
        return self.extent_utm[1] - self.extent_utm[0]

    @property
    def height(self):
        """Map height (along northing direction) in meters"""
        return self.extent_utm[3] - self.extent_utm[2]

    @property
    def res_easting(self):
        """Raster spatial resolution (m/px) along easting direction"""
        return self.width/self.shape[1]

    @property
    def res_northing(self):
        """Raster spatial resolution (m/px) along northing direction"""
        return self.height/self.shape[0]

    @property
    def shape(self):
        """Raster shape (num. pixels) along northing and easting directions"""
        return self.mosaic.shape  # same shape for all rasters

    @property
    def origin_latlon(self):
        """Latitude and longitude coordinates of map origin (degrees)"""
        return self._ORIGIN_LATLON

    @property
    def origin_utm(self):
        """UTM coordinates (easting,northing) of map origin, in meters"""
        return utm.from_latlon(*self._ORIGIN_LATLON)[0:2]

    @property
    def zone_utm(self):
        """UTM zone code of map"""
        return self._UTM_ZONE


if __name__=="__main__":
    pass