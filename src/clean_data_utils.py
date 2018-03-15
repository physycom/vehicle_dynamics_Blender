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
from scipy import constants
from scipy.linalg import norm
from scipy import cross, dot, arccos, arctan, cos, sin
import quaternion


def converts_measurement_units(df):
    """ Convert accelerations from g units to m/s^2 and angular velocities from degrees/s to radians/s

    :param df: Pandas dataframe
    """
    # TODO vectorize
    df[['ax', 'ay', 'az']] = df[['ax', 'ay', 'az']].apply(lambda x: x * constants.g)
    df[['gx', 'gy', 'gz']] = df[['gx', 'gy', 'gz']].apply(lambda x: np.deg2rad(x))


def normalize_timestamp(df):
    """ Normalize timestamp to make it begin from time 0

    :param df: Pandas dataframe
    """
    motion_start = df.iloc[0, 0]
    # TODO vectorize
    df['timestamp'] = df['timestamp'].apply(lambda x: x - motion_start)


def clear_gyro_drift(df):
    """ Remove gyroscope natural drift

    This function assumes the car is stationary in the first 40 seconds

    :param df: Pandas dataframe
    """
    remove_x = df.loc[0:40, 'gx'].mean()
    remove_y = df.loc[0:40, 'gy'].mean()
    remove_z = df.loc[0:40, 'gz'].mean()
    df.loc[:, 'gx'] = df.loc[:, 'gx'] - remove_x
    df.loc[:, 'gy'] = df.loc[:, 'gy'] - remove_y
    df.loc[:, 'gz'] = df.loc[:, 'gz'] - remove_z


def reduce_disturbance(df):
    """ Reduce data disturbance with a moving average

    :param df: Pandas dataframe
    """

    # windows dimension is 1/60 of dataframe rows count
    window_dimension = round(df.shape[0] / 60)
    df.loc[:, 'ax':'az'] = df.loc[:, 'ax':'az'].rolling(window=window_dimension).mean()
    df.loc[:, 'gx':'gz'] = df.loc[:, 'gx':'gz'].rolling(window=window_dimension).mean()
    index_to_drop = df.index[0:window_dimension]
    new_row_upper_limit = df.shape[0] - window_dimension
    df.drop(index=index_to_drop, inplace=True)
    df.index = range(0, new_row_upper_limit)


ax_threshold = 0.1
g_z_threshold = 0.01


def get_xy_bad_align_proof(df):
    """
    find the first time (if present) where x and y accelerations are over a threshold
    and angular speed around z is near or equal 0
    :param df: Pandas dataframe
    :return: rows where condition apply
    """
    return df[(abs(df['ax']) > ax_threshold)
              & (abs(df['ay']) > ax_threshold)
              & (abs(df['gz']) < g_z_threshold)]


def correct_xy_orientation(df):
    """ Detect bad position of sensor in the xy plane and correct reference frame

    :param df: Pandas dataframe
    """

    bad_align_proof = get_xy_bad_align_proof(df)
    # get first vector
    ax, ay = bad_align_proof.values[0, 1:3]
    # get angle and negate it to remove rotation
    angle = -arctan(float(ay) / float(ax))
    # rotate vector
    df['ax'] = cos(angle) * df['ax'] - sin(angle) * df['ay']
    df['ay'] = sin(angle) * df['ax'] + cos(angle) * df['ay']
    # TODO find if there are others times where the condition returns
    # raise a warning/exception
    # rotate from that time above


def correct_z_orientation(values, columns):
    """ Use gravity vector direction to align reference frame to correct z-axis

    Assumes the car is stationary for the first 40s and the gravity haven't been removed

    :param values: numpy array
    :param columns: dictionary of columns names and relative position in values parameter
    :return: numpy array of rotated vector values
    """
    x_g = values[0:40, columns['ax']].mean()
    y_g = values[0:40, columns['ay']].mean()
    z_g = values[0:40, columns['az']].mean()
    g = np.array((x_g, y_g, z_g))
    g_norm = norm(g)
    u = cross(g, (0, 0, 1))
    # rotation axis
    u_unit = u / norm(u)
    # rotate angle
    theta = arccos(dot(g, (0, 0, 1)) / g_norm)
    rotator = np.exp(quaternion.quaternion(*(theta * u_unit)) / 2)
    # columns_index = range(columns['ax'],columns['gz']+1)
    for x in values:
        x[2:5] = (rotator * quaternion.quaternion(*(x[2:5])) * ~rotator).components[1:]
        x[5:8] = (rotator * quaternion.quaternion(*(x[5:8])) * ~rotator).components[1:]
    return values
