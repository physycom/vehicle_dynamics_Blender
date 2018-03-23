"""
Various routines for cleaning input data.
The need of some of these functions is because of the input data format.
See <https://github.com/physycom/file_format_specifications/blob/master/formati_file.md#formato-dati-inerziali>
The need of some other functions is sensor brand and sensor installation dependent.

The purpose of all this module is get inertial data:
- accelerations on m/s^2
- angular velocities in radians/s
- more precise possible inertial measurements in the car's reference frame

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

import numpy as np
import quaternion
from scipy import constants
from scipy import cross, dot, arccos
from scipy.linalg import norm

from tests.test_fixtures import car_initial_stationary_time


def parse_input(df):
    """ Transform single dataframe to multiple numpy array each representing different physic quantity

    :param df: Pandas dataframe
    :return: 4 numpy array: 1xn timestamp, 1xn gps speed,
    3xn accelerations and 3xn angular velocities
    """
    accelerations = df[['ax', 'ay', 'az']].values.T
    angular_velocities = df[['gx', 'gy', 'gz']].values.T
    times = df['timestamp'].values.T
    gps_speed = df['speed'].values.T
    return times, gps_speed, accelerations, angular_velocities


def converts_measurement_units(gps_speed, accelerations, angular_velocities):
    """ Convert physics quantity measurement unit

    Convert accelerations from g units to m/s^2
    angular velocities from degrees/s to radians/s
    gps speed from km/h to m/s

    Applies inplace

    :param gps_speed: 1xn array of gps speed in km/h
    :param accelerations: 3xn array of acceleration in g unit
    :param angular_velocities: 3xn array of angular velocities in degrees/s
    """
    accelerations *= constants.g
    # multiply to degree to radians constant
    angular_velocities *= constants.degree
    # multiply to km/h -> m/s constant
    gps_speed *= constants.kmh


def normalize_timestamp(times):
    """ Normalize timestamp to make it begin from time 0

    :param times: 1xn numpy array of timestamps
    """
    times -= times[0]


def sign_inversion_is_necessary(velocities):
    """ Check if sign of input vector must be inverted by manufacture convention

    This function assumes that speed has been converted to m/s

    :param velocities: 3xn numpy array of velocities
    """
    speed_threshold = -4
    return any(velocities[0] < speed_threshold)


def clear_gyro_drift(angular_velocities):
    """ Remove gyroscope natural drift

    This function assumes the car is stationary in the first 10000 measurements

    :param angular_velocities: 3xn numpy array of angular velocities
    """
    angular_velocities[0] = angular_velocities[0] - angular_velocities[0, 0:car_initial_stationary_time].mean()
    angular_velocities[1] = angular_velocities[1] - angular_velocities[1, 0:car_initial_stationary_time].mean()
    angular_velocities[2] = angular_velocities[2] - angular_velocities[2, 0:car_initial_stationary_time].mean()
    return angular_velocities


def reduce_disturbance(times, vectors):
    """ Reduce data disturbance with a moving average

    The length of the window is calculated internally in function of vector length
    Some values at the beginning of the array will be dropped.

    :param times: 1xn numpy array of timestamps
    :param vectors: 3xn numpy array of whatever numeric
    :return 2 numpy vector: new times and new vector
    """

    # TODO dynamically find windows dimension for 0.5 s
    window_dimension = 20
    # use pandas because it has built in function of moving average
    # performance overhead is not much
    import pandas as pd
    df = pd.DataFrame(vectors.T)
    # overwrite dataframe with its moving average
    df = df.rolling(window=window_dimension, center=True).mean()
    # now there ara 0:windows_dimension nan rows at the beginning
    # drop these rows
    new_low_range = round(window_dimension / 2)
    new_upper_range = round(df.shape[0] - window_dimension / 2)
    # TODO change drop offset
    new_vector = df[new_low_range:new_upper_range].values.T
    new_times = times[new_low_range:new_upper_range]
    return new_times, new_vector


def correct_z_orientation(accelerations, angular_velocities):
    """ Use gravity vector direction to align reference frame to correct z-axis

    Assumes the car is stationary for the first 10000 times and the gravity haven't been removed

    :param accelerations: 3xn numpy array angular velocities
    :param angular_velocities: 3xn numpy array angular velocities
    :return: numpy arrays: rotated accelerations, rotated angular velocities
    """

    g = accelerations[:, 0:car_initial_stationary_time].mean(axis=1)
    g_norm = norm(g)
    u = cross(g, (0, 0, 1))
    # rotation axis
    u_unit = u / norm(u)
    # rotate angle
    theta = arccos(dot(g, (0, 0, 1)) / g_norm)
    rotator = np.exp(quaternion.quaternion(*(theta * u_unit)) / 2)
    rotated_accelerations = np.array(
        [(rotator * quaternion.quaternion(*acceleration_vector) * ~rotator).components[1:]
         for acceleration_vector in accelerations.T])
    rotated_angular_velocities = np.array(
        [(rotator * quaternion.quaternion(*angular_velocity) * ~rotator).components[1:]
         for angular_velocity in angular_velocities.T])
    return rotated_accelerations.T, rotated_angular_velocities.T
