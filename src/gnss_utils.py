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
from scipy import arctan2

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
    positions = np.zeros((3, coordinates.shape[1]))
    headings = np.zeros(coordinates.shape[1])
    # current position
    current = np.array([coordinates[0, 0], coordinates[1, 0], altitudes[0]])
    # iterate skipping first position that is zero
    for i, gnss_data in enumerate(zip(coordinates[:, 1:].T, altitudes[1:]), start=1):
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
        headings[i] = arctan2(delta_lat,delta_lon)
        # update current
        current[0] = lat
        current[1] = lon
        current[2] = alt
        # set position at step i
        positions[0, i] = positions[0, i - 1] + delta_lat
        positions[1, i] = positions[1, i - 1] + delta_lon
        positions[2, i] = positions[2, i - 1] + delta_alt
    return positions, headings


def get_velocities(times, positions):
    """
    Get array of speeds from position in cartesian system

    :param times: 1xn numpy array of timestamp
    :param positions: 3xn numpy array of position in cartesian system
    :return: 3xn velocites numpy array
    """

    # remove altitude because it's unreliable
    #positions = np.delete(positions, 2, axis=0)
    from scipy.misc import derivative
    from scipy.interpolate import interp1d
    positions_func = interp1d(x=times, y=positions, kind='quadratic', fill_value='extrapolate')
    speeds = np.array([derivative(positions_func, time) for time in times])
    return speeds.T

def get_accelerations(times, velocities):
    """
    Get array of acceleration from velocites in cartesian system

    :param times: 1xn numpy array of timestamp
    :param velocities: 3xn numpy array of velocities in cartesian system
    :return: 3xn numpy array of accelerations
    """

    # remove altitude because it's unreliable
    #positions = np.delete(positions, 2, axis=0)
    from scipy.misc import derivative
    from scipy.interpolate import interp1d
    velocities_func = interp1d(x=times, y=velocities, kind='quadratic', fill_value='extrapolate')
    accelerations = np.array([derivative(velocities_func, time) for time in times])
    return accelerations.T


def align_to_world(gnss_position, vectors, stationary_times):
    """
    Align accelerations to world system (x axis going to east, y to north)

    :param gnss_position: 3xn numpy array. positions from gnss data
    :param vectors: 3xn numpy array
    :param stationary_times: list of tuples
    :return: 3xn numpy array of rotated accelerations
    """

    # iterate until a motion time (not stationary) is found
    i = 0
    motion_time_start = 0
    while i < len(stationary_times):
        motion_time_end = stationary_times[i][0]
        # motion time must be with more than 100 elements
        if (motion_time_end - motion_time_start > 100):
            break
        else:
            motion_time_start = stationary_times[i][1]
            i += 1
    # get mean vector from first 100 vector of motion time
    gnss_start = gnss_position[:, motion_time_start:motion_time_start + 100].mean(axis=1)
    vector_start = vectors[:, motion_time_start:motion_time_start + 100].mean(axis=1)
    from scipy import sin, cos, arctan2
    # get angle of rotation
    angle_gnss = arctan2(gnss_start[1], gnss_start[0])
    ancle_vector = arctan2(vector_start[1],vector_start[0])
    rotation_angle = angle_gnss-ancle_vector
    message = "Rotation vector to {} degrees to align to world".format(np.rad2deg(rotation_angle))
    print(message)
    new_vectors = vectors.copy()
    # rotate vector in xy plane
    new_vectors[0] = cos(rotation_angle) * vectors[0] - sin(rotation_angle) * vectors[1]
    new_vectors[1] = sin(rotation_angle) * vectors[0] + cos(rotation_angle) * vectors[1]
    return new_vectors
