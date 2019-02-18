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
import math
from quaternion import quaternion
from src.constants import g, kmh, degree_to_radians, pi

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

def get_stationary_times(gps_speed):
    """ Returns list of index where the gps speed is near zero

    :param gps_speed: 1xn numpy array of gps speed in m/s
    :return: list of tuples, each one with start and final timestamp of a stationary time index
    """

    speed_threshold = 1e-15  # 0.2 m/s
    stationary_times = []
    # repeat until at least a stationary time is found
    while (len(stationary_times)==0):
        # TODO this is very important find a way to smart finding a good value for this
        min_stationary_time_length = 10
        # Find times where gps speed is inside threshold
        # Didn't use np.nonzero because i needed contiguous slices and also check on length of slice
        boolean_vect = np.bitwise_and(gps_speed > -speed_threshold, gps_speed < speed_threshold)
        import math
        # set a value that can't exist as index
        first_true = math.inf
        last_true = math.inf
        for i, x in enumerate(boolean_vect):
            if first_true == math.inf and x is np.bool_(True):
                # enter stationary time mode
                first_true = i
            if first_true != math.inf and x is np.bool_(True):
                # add a timestamp to slice
                last_true = i
            if first_true != math.inf and last_true != math.inf and x is np.bool_(False):
                # end slice
                # if slice length is greater than a minimum length
                if last_true - first_true > min_stationary_time_length:
                    # add slice to stationary times list
                    stationary_times.append((first_true, last_true))
                # reset
                first_true = math.inf
                last_true = math.inf
        # handle case where stationary times not ends before data ends
        if first_true != math.inf and last_true != math.inf and (last_true - first_true) > min_stationary_time_length:
            stationary_times.append((first_true, last_true))
        # TODO raise exception if there are no stationary times
        # increase speed threshold in case stationary times are not found
        speed_threshold += 0.1
    return stationary_times


def converts_measurement_units(accelerations, angular_velocities, gps_speed=None, coordinates=None, heading=None):
    """ Convert physics quantity measurement unit

    Convert accelerations from g units to m/s^2
    coordinates from degrees to radians
    angular velocities from degrees/s to radians/s
    gps speed from km/h to m/s

    Applies inplace

    :param gps_speed: 1xn array of gps speed in km/h
    :param accelerations: 3xn array of acceleration in g unit
    :param angular_velocities: 3xn array of angular velocities in degrees/s
    :param coordinates: optional 2xn array of coordinates in geographic coordinate system
    """
    accelerations *= g
    if coordinates is not None:
        # multiply to degree to radians constant
        coordinates *= degree_to_radians
    # multiply to degree to radians constant
    angular_velocities *= degree_to_radians
    if gps_speed is not None:
        # multiply to km/h -> m/s constant
        gps_speed *= kmh
    if heading is not None:
        heading *= degree_to_radians


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


def clear_gyro_drift(angular_velocities, stationary_times):
    """ Remove gyroscope natural offset

    This function assumes the car is stationary in the first 10000 measurements

    :param angular_velocities: 3xn numpy array of angular velocities
    :param stationary_times: list of tuples (start,end)
    """

    # get offset on first stationary time
    main_offset = angular_velocities[:, stationary_times[0][0]:stationary_times[0][1]].mean(axis=1)
    # remove on all data
    angular_velocities = (angular_velocities.T - main_offset).T
    # for remaining stationary times
    for stationary_time in stationary_times[1:]:
        start = stationary_time[0]
        end = stationary_time[1]
        # offset can now be changed by heat, remove only from start time of stationary time to end of data
        offset = angular_velocities[:, start:end].mean(axis=1)
        angular_velocities[:, start:] = (angular_velocities[:, start:].T - offset).T
    return angular_velocities


def reduce_disturbance(times, vectors, window_dimension):
    """ Reduce data disturbance with a moving average

    The length of the window is calculated internally in function of vector length
    Some values at the beginning of the array will be dropped.

    :param times: 1xn numpy array of timestamps
    :param vectors: 3xn numpy array of whatever numeric
    :param window_dimension: int rolling average window dimension
    :return 2 numpy vector: new times and new vector
    """

    # TODO dynamically find windows dimension for 0.5 s
    window_dimension = 100
    # iterating moving average is the same of weighting it like a gaussian
    for j in range(10):
        # overwrite dataframe with its moving average
        array = np.zeros(vectors.shape)
        for i in range (0,window_dimension):
            sliced_vec = vectors[:,i:]
            array[:,:sliced_vec.shape[1]] += sliced_vec
        # centered moving average
        array[:,round(window_dimension/2):] /= window_dimension
    # now there ara 0:windows_dimension nan rows at the beginning
    # drop these rows
    new_low_range = round(window_dimension/2)
    #new_low_range = math.floor(window_dimension / 2)
    new_upper_range = math.floor(vectors.shape[1] - round(window_dimension/2))
    # TODO change drop offset
    times = times[new_low_range:new_upper_range]
    vectors = array[:,new_low_range:new_upper_range]
    return times, vectors

def manual_sensor_rotation_fix(accelerations, angular_velocities, angle_x, angle_y, angle_z):
    """
    Routine to rotate all inertial vector by euler angle. Useful for fixing sensor bad alignment if known.

    :param accelerations: 3xn array
    :param angular_velocities: 3xn array
    :param angle_x: euler angle in radians
    :param angle_y: euler angle in radians
    :param angle_z: euler angle in radians
    :return:
    """

    # create 3 matrix to compose rotation matrix
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(angle_x), -math.sin(angle_x)],
                    [0, math.sin(angle_x), math.cos(angle_x)]
                    ])

    R_y = np.array([[math.cos(angle_y), 0, math.sin(angle_y)],
                    [0, 1, 0],
                    [-math.sin(angle_y), 0, math.cos(angle_y)]
                    ])

    R_z = np.array([[math.cos(angle_z), -math.sin(angle_z), 0],
                    [math.sin(angle_z), math.cos(angle_z), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))
    # rotation by rotation matrix is the fastest one
    accelerations[...] = np.matmul(R, accelerations)
    angular_velocities[...] = np.matmul(R, accelerations)


# threshold above which acceleration along x and y axis are considered
axy_threshold = 0.2
max_axy_threshold = 10
# threshold above which acceleration along x and y axis are considered
g_z_threshold = 0.06


def get_xy_bad_align_count(accelerations, angular_velocities):
    """
    find records (if present) where x and y accelerations are over a threshold
    and angular speed around z is near or equal 0

    :param accelerations: 3xn numpy array angular velocities
    :param angular_velocities: 3xn numpy array angular velocities
    :return: rows where condition apply
    """
    # get all 4 +- combinations
    x1, x2, x3, x4 = get_bad_alignment_vectors(accelerations, angular_velocities)
    return sum(map(lambda x: x.shape[1] if len(x) > 0 else 0, [x1, x2, x3, x4]))


def get_bad_alignment_vectors(accelerations, angular_velocities):
    # boolean array of array position where x accelerations are above threshold
    ax_over_threshold = np.logical_and(accelerations[0] > axy_threshold, accelerations[0] < max_axy_threshold)
    ax_below_threshold = np.logical_and(accelerations[0] < -axy_threshold, accelerations[0] > -max_axy_threshold)
    # boolean array of array position where y accelerations are above threshold
    ay_over_threshold = np.logical_and(accelerations[1] > axy_threshold, accelerations[1] < max_axy_threshold)
    ay_below_threshold = np.logical_and(accelerations[1] < -axy_threshold, accelerations[1] > -max_axy_threshold)
    # boolean array of array position where z angular speed are below threshold
    gx_below_threshold = abs(angular_velocities[2]) < g_z_threshold

    def filter_contiguous(vector, conditions):
        """ Finds 100 contiguous vectors where all the given conditions are true

        :param vector: numpy columns vector
        :param conditions: list of boolean arrays with same length of vector
        :return vector: at least 100 long subset of vector where conditions apply
        """
        import math
        start = math.inf
        end = math.inf
        from functools import reduce
        for i, _ in enumerate(vector.T):
            conditions_on_i = map(lambda x: x[i], conditions)
            # logical AND on all conditions list
            condition_value = reduce(lambda acc, x: acc & x, conditions_on_i)
            if start == math.inf and condition_value:
                start = i
            if start != math.inf and condition_value:
                end = i
            if start != math.inf and end != math.inf and not condition_value:
                if end - start > 100:
                    break
                start = math.inf
                end = math.inf
        if start != math.inf and end != math.inf:
            return vector[:, start:end]
        else:
            return np.array([])

    # operate logical AND element-wise to get elements
    x1 = filter_contiguous(accelerations, [ax_over_threshold, ay_over_threshold, gx_below_threshold])
    x2 = filter_contiguous(accelerations, [ax_below_threshold, ay_over_threshold, gx_below_threshold])
    x3 = filter_contiguous(accelerations, [ax_over_threshold, ay_below_threshold, gx_below_threshold])
    x4 = filter_contiguous(accelerations, [ax_below_threshold, ay_below_threshold, gx_below_threshold])
    return x1, x2, x3, x4


def correct_xy_orientation(accelerations, angular_velocities):
    """ Detect bad position of sensor in the xy plane and correct reference frame

    :param accelerations: 3xn numpy array angular velocities
    :param angular_velocities: 3xn numpy array angular velocities

    :return accelerations: 3xn numpy array
    """

    def rotatexy(bad_align_proof):
        new_accelerations = accelerations.copy()
        if len(bad_align_proof) > 0 and bad_align_proof.shape[1] > 0:
            # get first vector
            vec = bad_align_proof.mean(axis=1)
            # get angle and negate it to remove rotation
            angle = -np.arctan2(vec[1], vec[0])
            if (vec[1] > 0 and vec[0] < 0):
                # TODO check if this case is necessary (is out of coverage)
                angle = pi + angle
            elif (vec[1] < 0 and vec[0] < 0):
                angle = -(pi + angle)
            # use new var instead of inplace so when we rotate y we don't use the rotated x but the old one
            new_accelerations[0] = np.cos(angle) * accelerations[0] - np.sin(angle) * accelerations[1]
            new_accelerations[1] = np.sin(angle) * accelerations[0] + np.cos(angle) * accelerations[1]
        # now set new arrays
        return get_xy_bad_align_count(new_accelerations, angular_velocities), new_accelerations

    print("initial bad align sum {}".format(get_xy_bad_align_count(accelerations, angular_velocities)))
    # get bad align vector for all +- combinations
    x1, x2, x3, x4 = get_bad_alignment_vectors(accelerations, angular_velocities)
    # get best vector than minimize sum of bad align vectors after rotations
    best_bad_vector = min([x1, x2, x3, x4], key=lambda x: rotatexy(x)[0])
    # rotate accelerations
    _, accelerations = rotatexy(best_bad_vector)
    print("final bad align sum {}".format(get_xy_bad_align_count(accelerations, angular_velocities)))
    return accelerations

    # TODO find if there are others times where the condition returns
    # raise a warning/exception
    # rotate from that time above


def correct_z_orientation(accelerations, angular_velocities, stationary_times):
    """ Use gravity vector direction to align reference frame to correct z-axis

    Assumes the car is stationary for the first 10000 times and the gravity haven't been removed

    :param accelerations: 3xn numpy array angular velocities
    :param angular_velocities: 3xn numpy array angular velocities
    :param stationary_times: list of tuples (start,end)
    :return: numpy arrays: rotated accelerations, rotated angular velocities
    """

    # get value of g in all stationary times
    g = np.mean(np.concatenate(
        [accelerations[:,stationary_time[0]:stationary_time[1]] for stationary_time in stationary_times],axis=1),axis=1)
    def align_from_g_vector(accelerations, angular_velocities, g):
        g_norm = np.linalg.norm(g)
        u = np.cross(g, (0, 0, 1))
        # rotation axis
        u_unit = u / np.linalg.norm(u)
        # rotate angle
        theta = np.arccos(np.dot(g, (0, 0, 1)) / g_norm)
        print("rotating vectors of "+str(np.rad2deg(theta))+" degrees align to z")
        rotator = np.exp(quaternion(*(theta * u_unit)) / 2)
        rotated_accelerations = np.array(
            [(rotator * quaternion(*acceleration_vector) * ~rotator).components[1:]
             for acceleration_vector in accelerations.T])
        rotated_angular_velocities = np.array(
            [(rotator * quaternion(*angular_velocity) * ~rotator).components[1:]
             for angular_velocity in angular_velocities.T])
        return rotated_accelerations.T, rotated_angular_velocities.T

    accelerations, angular_velocities = align_from_g_vector(accelerations, angular_velocities, g)

    # for the remaining stationary times
    for stationary_time in stationary_times[1:]:
        # calculate bad align angle
        g = accelerations[:, stationary_time[0]:stationary_time[1]].mean(axis=1)
        bad_align_angle = np.arccos(np.dot(g, (0, 0, 1)) / np.linalg.norm(g))
        # if the bad align angle is greater than 2 degrees
        if bad_align_angle > np.deg2rad(10):
            # print a warning
            import warnings
            message = " \n Found additional bad z axis of {} degrees alignment at time {} , " \
                      "realigning from now  \n".format(np.rad2deg(bad_align_angle), stationary_time[0])
            warnings.warn(message)
            # re-align
            accelerations, angular_velocities = align_from_g_vector(accelerations, angular_velocities, g)
    return accelerations, angular_velocities
