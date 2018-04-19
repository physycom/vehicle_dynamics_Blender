#!/usr/bin/env python
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
import quaternion


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


def simps_integrate(times, vectors, initial=np.zeros(3), adjust_data=None, adjust_frequency=None):
    """
    Simpson integration with irregularly-spaced data
    Optional initial data reset with custom frequency

    Method from paper https://scholarworks.umt.edu/cgi/viewcontent.cgi?article=1319&context=tme
    Works both with even and odd vectors

    :param times: 1xn np array of timestamps
    :param vectors: 3xn np vector to integrate
    :param initial: 3x1 np array integration initial value
    :param adjust_data: 3xn numpy array. Data to reset to each adjust frequency times.
    :param adjust_frequency: 3xn numpy array. Frequency of adjust operations.
    :return: 3xn np array vector integrated
    """

    stop = vectors.shape[1]
    # create vector to keep results
    result_vectors = np.zeros((3, stop))
    # if vector is even
    if stop % 2 == 0:
        # makes main integration on an odd number of elements
        stop = stop - 1
    # create vector to keep current progress value
    current = np.reshape(initial, (3, 1))
    # save current in first result position
    result_vectors[:, 0] = current.T
    # iterate through vector to integrate
    for i in range(0, stop - 2):
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
        delta = np.array([integrator(x[0], x[1]) for integrator in integrators])
        # reshape delta vector to make sum with current
        delta = np.reshape(delta, (3, 1))

        if adjust_data is not None and adjust_frequency is not None and i % adjust_frequency == 0:
            current[0] = adjust_data[0, i]
            current[1] = adjust_data[1, i]
            current = np.reshape(current, (3, 1))

        current = current + delta
        # save it in the next position
        result_vectors[:, i + 1] = current.T
    # fill last element element with already calculated integrator
    # get integral value of "last part" of the parabola
    delta = np.array([integrator(x[1], x[2]) for integrator in integrators])
    # reshape delta vector to make sum with current
    delta = np.reshape(delta, (3, 1))
    current = current + delta
    # set current velocity in last/second-last element (depends on odd/even vectors)
    result_vectors[:, i + 2] = current.T
    # if the vector is even the last element is still zero
    if all(result_vectors[:, -1]) == 0:
        # interpolate a parabola for last three elements
        x = times[-3:]
        y = vectors[:, -3:]
        integrators = get_parab_integrator(x, y)
        # get integral value of "last part" of the parabola
        delta = np.array([integrator(x[1], x[2]) for integrator in integrators])
        # reshape delta vector to make sum with current
        delta = np.reshape(delta, (3, 1))
        # save in last position
        result_vectors[:, -1] = (current + delta).T
    return result_vectors


def rotate_accelerations(times, accelerations, angular_velocities):
    # integrate angular velocities to get angular positions
    angular_positions = trapz_integrate(times, angular_velocities)
    for i in range(accelerations.shape[1] - 1):
        # create rotation quaternion from angular position at step i
        rotator = np.exp(quaternion.quaternion(*np.asarray(angular_positions[:, i])) / 2)
        # create pure quaternion from acceleration vector at step i
        acc_to_rotate = quaternion.quaternion(*np.asarray(accelerations[:, i]))
        #rotate acceleration quaternion with rotation quaternion
        accelerations[:, i] = (rotator * acc_to_rotate * ~rotator).components[1:4]
    return accelerations
