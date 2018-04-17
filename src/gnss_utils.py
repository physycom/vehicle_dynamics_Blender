"""
Various routines to handle GNSS data

This file is part of inertial_to_blender project,
a Blender simulation generator from inertial sensor data on cars.

Copyright (C) 2018  Federico Bertani
Author: Federico Bertani
Credits: Federico Bertani, Stefano Sinigardi, Alessandro Fabbri, Nico Curti

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""

from math import cos

import numpy as np


def get_positions(coordinates, altitudes):
    """
    Convert gss data from geographic coordinate system to cartesian

    :param coordinates: 2xn numpy array of coordinates (lat,lon)
    :param altitudes: 1xn numpy array of altitudes
    :return: 3xn numpy array of position in cartesian system
    """
    earth_radius = 6371000
    # create empty array for final positions
    positions = np.zeros((3, coordinates.shape[0]))
    # current position
    current = np.array([coordinates[0, 0], coordinates[0, 1], altitudes[0]])
    # iterate skipping first position that is zero
    for i, gnss_data in enumerate(zip(coordinates[1:], altitudes[1:]), start=1):
        # get data from zipped tuple
        lat = gnss_data[0][0]
        lon = gnss_data[0][1]
        alt = gnss_data[1]
        # current is the previous record
        # use relationship between central angle and arc to calculate delta lat
        delta_lat = earth_radius * (lat - current[0])
        # use same formula but with earth horizontal radius moved to latitude
        delta_lon = earth_radius * cos(lat) * (lon - current[1])
        delta_alt = alt - current[2]
        # update current
        current[0] = lat
        current[1] = lon
        current[2] = alt
        # set position at step i
        positions[0, i] = positions[0, i - 1] + delta_lat
        positions[1, i] = positions[1, i - 1] + delta_lon
        positions[2, i] = positions[2, i - 1] + delta_alt
    return positions
