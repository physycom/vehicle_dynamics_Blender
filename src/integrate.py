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
from scipy.integrate import simps, trapz


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


def simps_integrate(times, accelerations, initial=np.zeros(3)):
    stop = accelerations.shape[1]
    velocities = np.zeros((3, stop))
    stop = stop - 1 if (stop % 2 == 0) else stop
    current_velocity = np.reshape(initial, (3, 1))
    for i in range(0, stop, 2):
        velocities[:, i:i + 3] = current_velocity
        accelerations_local = accelerations[:, i:i + 3]
        times_local = times[i:i + 3]
        delta_v = simps(y=accelerations_local, x=times_local, axis=1)
        delta_v = np.reshape(delta_v, (3, 1))
        current_velocity = current_velocity + delta_v
    if np.all(velocities[:, -1:] == 0):
        velocities[:, :-1] = velocities[:, -2] + trapz(times[-2:], accelerations[:, -2:])
    return velocities


def rotate_accelerations(times, accelerations, angular_velocities):
    # integrate angular velocities to get angular positions
    angular_positions = trapz_integrate(times, angular_velocities)
    for i in range(accelerations.shape[1] - 1):
        # create rotation quaternion from angular position at step i
        rotator = np.exp(quaternion.quaternion(*np.asarray(angular_positions[:, i])) / 2)
        # create pure quaternion from acceleration vector at step i
        acc_to_rotate = quaternion.quaternion(*np.asarray(accelerations[:, i]))
        # rotate acceleration quaternion with rotation quaternion
        accelerations[:, i] = (rotator * acc_to_rotate * ~rotator).components[1:4]
    return accelerations
