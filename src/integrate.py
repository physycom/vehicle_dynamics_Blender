"""
This module expose routines to create car path from accelerometer
and gyroscope data by numerical integration.


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
from pyquaternion import Quaternion



def quad_integrate(times, vector, initial=np.zeros(3)):
    current = initial
    result_vector = np.zeros((3, vector.shape[1]))
    result_vector[:, 0] = initial
    for i in range(vector.shape[1] - 1):
        dt = times[i + 1] - times[i]
        dv = vector[:, i + 1] * dt
        current = current + dv
        result_vector[:, i + 1] = current
    return result_vector


def trapz_integrate(times, vector, initial=np.zeros(3)):
    current = initial
    result_vector = np.zeros((3, vector.shape[1]))
    result_vector[:, 0] = initial
    for i in range(vector.shape[1] - 1):
        dt = times[i + 1] - times[i]
        dv = ((vector[:, i] + vector[:, i + 1]) * dt) / 2
        current = current + dv
        result_vector[:, i + 1] = current
    return result_vector

def trapz_integrate_delta(times, vector):
    # multiplying by 0.5 faster than dividing by two
    return ((vector[:,:-1] + vector[:,1:]) * (times[:-1] - times[1:])) * 0.5


def simps_integrate_delta(times, vectors):
    """
    Simpson integration with irregularly-spaced data
    Returns delta vector

    Method from paper https://scholarworks.umt.edu/cgi/viewcontent.cgi?article=1319&context=tme
    Works both with even and odd vectors

    :param times: 1xn np array of timestamps
    :param vectors: 3xn np vector to integrate
    :return: 3xn np array vector of deltas
    """

    rows = vectors.shape[0]
    columns = vectors.shape[1]
    # create vector to keep results
    deltas = np.zeros((rows, columns))
    # if vector is even
    if columns % 2 == 0:
        # makes main integration on an odd number of elements
        columns = columns - 1
    integrators = None
    # iterate through vector to integrate
    for i in range(0, columns - 2):
        # create (x_i,y_i) points
        x = times[i:i + 3]
        # 3x3 array
        vector_locals = vectors[:, i:i + 3]

        def get_parab_integrator(x, vector_locals):
            """ Returns array of functions

            Each function integrates a interpolated parabola in (x,vector_locals[i])
            """

            integrators = []
            for i, y in enumerate(vector_locals):
                # create matrix to solve linear system
                matrix = np.array([
                    [x[0] ** 2, x[0], 1],
                    [x[1] ** 2, x[1], 1],
                    [x[2] ** 2, x[2], 1],
                ])
                # solve linear system with matrix inversion and dot product
                A, B, C = np.dot(np.linalg.inv(matrix), y)

                # use classes because closure didn't works
                class Integrator:

                    def __init__(self, A, B, C):
                        self.A = A
                        self.B = B
                        self.C = C

                    def integrator_fun(self, x1, x3):
                        return self.A / 3 * (x3 ** 3 - x1 ** 3) + self.B / 2 * (x3 ** 2 - x1 ** 2) + self.C * (x3 - x1)

                integrators.append(Integrator(A, B, C).integrator_fun)
            return integrators

        integrators = get_parab_integrator(x, vector_locals)
        # for each integrator function get integral value only of the "first part" of the parabola
        deltas[:, i + 1] = np.array([integrator(x[0], x[1]) for integrator in integrators])
    # fill last element element with already calculated integrator
    # get integral value of "last part" of the parabola
    deltas[:, i + 2] = np.array([integrator(x[1], x[2]) for integrator in integrators])
    # if the vector is even the last element is still zero
    if all(deltas[:, -1]) == 0:
        # interpolate a parabola for last three elements
        x = times[-3:]
        y = vectors[:, -3:]
        integrators = get_parab_integrator(x, y)
        # get integral value of "last part" of the parabola
        deltas[:, -1] = np.array([integrator(x[1], x[2]) for integrator in integrators])
    return deltas

def cumulative_integrate(times, vectors, initial=None, delta_integrate_func = simps_integrate_delta, adjust_data=None, adjust_frequency=None):
    """
    Optional initial data reset with custom frequency

    :param times: 1xn np array of timestamps
    :param vectors: 3xn np vector to integrate
    :param initial: 3x1 np array integration initial value
    :param adjust_data: 3xn numpy array. Data to reset to each adjust frequency times.
    :param adjust_frequency: 3xn numpy array. Frequency of adjust operations.
    :return: 3xn numpy array integrated vectors

    """
    rows = vectors.shape[0]
    columns = vectors.shape[1]
    delta_vectors = delta_integrate_func(times,vectors)
    # create vector to keep results
    result_vectors = np.zeros((rows,columns))
    # save initial in first result position
    result_vectors[:, 0] = np.zeros(rows) if (initial is None) else np.reshape(initial,(1,rows))
    # iterate delta_vector skipping first position
    for i,delta_vector in enumerate(delta_vectors[:,1:].T,1):
        # if adjust is needed
        if adjust_data is not None and adjust_frequency is not None and i % adjust_frequency == 0:
            # reset result vectors
            if rows == 3:
                # do not adjust z-axis (altitude is not reliable)
                result_vectors[:-1, i] = \
                    adjust_data[:-1, i]*0.01 + (result_vectors[:-1, i - 1] + delta_vectors[:-1, i])*0.99
            elif rows == 1:
                result_vectors[0, i] = \
                    adjust_data[0, i]*0.01 + (result_vectors[0, i - 1] + delta_vectors[0, i])*0.99
        else:
            # cumulative sum
            result_vectors[:,i] = result_vectors[:,i-1] + delta_vectors[:,i]
    return result_vectors


def rotate_accelerations(times, accelerations, angular_velocities, initial_angular_position=np.array([0, 0, 0])):
    """
    Integrate angular velocities and rotate acceleration vector accondingly.
    Moves from local frame of reference to laboratory one.

    :param times: 1xn numpy array of timestamp
    :param accelerations: 3xn numpy array of accelerations
    :param angular_velocities: 3xn numpy array of angular velocities in rad/s
    :param initial_angular_positiLebensrumon: 1x3 numpy array containing initial angular position vector
    :return: 2 numpy array: 3xn acceleration vector and 4xn angular position as quaternion
    """

    # integrate angular_velocities to get a delta theta vector
    delta_thetas = simps_integrate_delta(times,angular_velocities)
    initial_quaternion = Quaternion.exp(Quaternion(vector=initial_angular_position)/2)
    # create quaternion representing angular position (angular position = rotation_versor * rotation_angle)
    quaternions = np.array([Quaternion.exp(Quaternion(vector=delta_theta)/2)
         for delta_theta in delta_thetas[:,1:].T])
    # cant use np.cumprod becuase in quaternion to rotate first by q1 then by q2 the aggregated quaternion is q2q1 not q1q2
    # so use reduce
    from functools import reduce
    quaternions = reduce(lambda array, element: [*array, element * array[-1]], quaternions, [initial_quaternion])
    accelerations = np.array(list(map(lambda x:x[1].rotate(x[0]),zip(accelerations.T,quaternions))))
    angular_positions = np.array([quaternion.elements for quaternion in quaternions])
    return accelerations.T, angular_positions.T