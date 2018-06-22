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
from pyquaternion import Quaternion

import numpy as np


def get_positions(coordinates, altitudes):
    """
    Convert gss data from geographic coordinate system to cartesian

    :param coordinates: 2xn numpy array of coordinates (lat,lon)
    :param altitudes: 1xn numpy array of altitudes
    :return: 2 numpy array: 3xn numpy array of position in cartesian system 1xn heading as array of angles
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
        headings[i] = arctan2(delta_lat, delta_lon)
        # update current
        current[0] = lat
        current[1] = lon
        current[2] = alt
        # set position at step i
        positions[0, i] = positions[0, i - 1] + delta_lon
        positions[1, i] = positions[1, i - 1] + delta_lat
        positions[2, i] = positions[2, i - 1] + delta_alt
    return positions, headings


def get_velocities(times, positions, win_size=320):
    """
    Get array of speeds from position in cartesian system

    Increasing windows size improve performance but reduce precision.

    :param times: 1xn numpy array of timestamp
    :param positions: 3xn numpy array of position in cartesian system
    :param win_size: int how often calculate velocity
    :return: 2xn velocities numpy array
    """
    # TODO this is very basic numerical differentiation, can be improved
    # remove altitude because it's unreliable
    positions = np.delete(positions, 2, axis=0)
    # create array
    velocities = np.zeros((2, positions.shape[1]))
    # iterate starting from win size until end of array, with win_size step
    for i in range(win_size, positions.shape[1], win_size):
        delta_x = positions[0, i] - positions[0, i - win_size]
        delta_y = positions[1, i] - positions[1, i - win_size]
        delta_t = times[i] - times[i - win_size]
        velocities[0, i - win_size:i] = delta_x / delta_t
        velocities[1, i - win_size:i] = delta_y / delta_t
    return velocities


def get_accelerations(times, velocities, win_size=320):
    """
    Get array of acceleration from velocities in cartesian system

    :param times: 1xn numpy array of timestamp
    :param velocities: 2xn numpy array of velocities in 2d cartesian system
    :param win_size: int how often calculate acceleration
    :return: 2xn numpy array of accelerations
    """

    accelerations = np.zeros((2, velocities.shape[1]))
    for i in range(win_size, velocities.shape[1], win_size):
        delta_x = velocities[0, i] - velocities[0, i - win_size]
        delta_y = velocities[1, i] - velocities[1, i - win_size]
        delta_t = times[i] - times[i - win_size]
        accelerations[0, i - win_size:i] = delta_x / delta_t
        accelerations[1, i - win_size:i] = delta_y / delta_t
    return accelerations


def align_to_world(gnss_position, vectors, stationary_times, angular_positions):
    """
    Align accelerations to world system (x axis going to east, y to north)

    :param gnss_position: 3xn numpy array. positions from gnss data
    :param vectors: 3xn numpy array
    :param stationary_times: list of tuples
    :param angular_positions:
    :return: 2 numpy array: 3xn numpy array of rotated accelerations and 4xn anguar positions as quaternions
    """

    # iterate until a motion time (not stationary) is found
    i = 0
    motion_time_start = 0
    size = stationary_times[0][0]
    motion_time_end = stationary_times[0][0]
    # find the first 100 records in motion times
    while size < 100:
        i += 1
        increase = stationary_times[i][0] - stationary_times[i - 1][1]
        if increase > 100:
            # TODO handle case where dataset is smaller
            motion_time_end = stationary_times[i - 1][0] + 100
            break
        else:
            size += increase
    # get mean vector from first 100 vector of motion time
    gnss_start = gnss_position[:, motion_time_start:motion_time_end].mean(axis=1)
    vector_start = vectors[:, motion_time_start:motion_time_start + 100].mean(axis=1)
    from scipy import sin, cos, arctan2
    # get angle of rotation
    angle_gnss = arctan2(gnss_start[1], gnss_start[0])
    angle_vector = arctan2(vector_start[1], vector_start[0])
    rotation_angle = angle_gnss - angle_vector
    message = "Rotation vector to {} degrees to align to world".format(np.rad2deg(rotation_angle))
    print(message)
    new_vectors = vectors.copy()
    quaternions = [Quaternion(elements) for elements in angular_positions.T]
    rotation_quaternion = Quaternion.exp(Quaternion(vector=rotation_angle * np.array([0, 0, 1])) / 2)
    angular_positions = np.array([(quaternion * rotation_quaternion).elements for quaternion in quaternions])
    # rotate vector in xy plane
    new_vectors[0] = cos(rotation_angle) * vectors[0] - sin(rotation_angle) * vectors[1]
    new_vectors[1] = sin(rotation_angle) * vectors[0] + cos(rotation_angle) * vectors[1]
    return new_vectors, angular_positions.T
