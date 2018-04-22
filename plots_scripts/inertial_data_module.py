#!/usr/bin/env python
"""
Plot integrated distance traveled
This is abstract from rotation probles.

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

import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import norm

from src.clean_data_utils import converts_measurement_units, reduce_disturbance, \
    clear_gyro_drift, correct_z_orientation, normalize_timestamp, \
    get_stationary_times
from src.gnss_utils import get_positions, get_velocities, get_accelerations
from src.input_manager import parse_input, InputType
from src.integrate import simps_integrate

if __name__ == '__main__':

    window_size = 20

    # for benchmarking
    import time

    start_time = time.time()

    parking_fullinertial_unmod = 'tests/test_fixtures/parking.tsv'
    times, coordinates, altitudes, gps_speed , accelerations, angular_velocities = parse_input(parking_fullinertial_unmod, [InputType.UNMOD_FULLINERTIAL])
    converts_measurement_units(accelerations, angular_velocities, gps_speed, coordinates)

    # GNSS data handling
    gnss_positions, headings = get_positions(coordinates, altitudes)
    gnss_distance = norm(np.array([gnss_positions[:,i]-gnss_positions[:,i-1]
             for i,x in enumerate(gnss_positions[:,1:].T,1)]),axis=1).cumsum()
    gnss_distance = np.reshape(gnss_distance,(1,len(gnss_distance)))
    real_velocities = get_velocities(times, gnss_positions)

    real_acc = get_accelerations(times, real_velocities)
    real_acc_module = norm(real_acc,axis=0)
    real_speeds = abs(real_velocities).sum(axis=0)

    stationary_times = get_stationary_times(real_speeds)
    # reduce accelerations disturbance
    times, accelerations = reduce_disturbance(times, accelerations, window_size)
    # reduce angular velocities disturbance
    _, angular_velocities = reduce_disturbance(times, angular_velocities, window_size)
    real_acc = real_acc[:,round(window_size/2):-round(window_size/2)]
    real_velocities = real_velocities[:,round(window_size/2):-round(window_size/2)]
    gnss_positions = gnss_positions[:,round(window_size/2):-round(window_size/2)]
    gnss_distance = gnss_distance[:,round(window_size/2):-round(window_size/2)]

    angular_velocities = clear_gyro_drift(angular_velocities, stationary_times)
    normalize_timestamp(times)
    accelerations, angular_velocities = correct_z_orientation(accelerations, angular_velocities, stationary_times)
    # remove g
    accelerations[2] -= accelerations[2, stationary_times[0][0]:stationary_times[0][-1]].mean()
    accelerations_module = norm(accelerations,axis=0)
    accelerations_module = np.reshape(accelerations_module, (1, len(accelerations_module)))

    real_velocities_module = norm(real_velocities,axis=0)
    real_velocities_module = np.reshape(real_velocities_module, (1, len(real_velocities_module)))
    initial_speed = np.array([[gps_speed[0]],[0],[0]])
    initial_speed_module = norm(initial_speed,axis=0)

    velocities_module = simps_integrate(times, accelerations_module, initial_speed_module)

    correct_velocities_module = simps_integrate(times, accelerations_module, initial_speed_module, adjust_data=real_velocities_module,
                                         adjust_frequency=40)

    #if sign_inversion_is_necessary(correct_velocities):
    #    accelerations *= -1
    #    velocities_module

    distance = simps_integrate(times, velocities_module)

    print("Execution time: %s seconds" % (time.time() - start_time))

    # plotting
    fig, ax1 = plt.subplots()
    ax1.plot(real_acc_module.T, label='gnss accellerations')
    ax1.plot(accelerations_module.T, label='int accellerations module')
    ax1.legend()
    ax2 = ax1.twinx()
    ax2.plot(abs((real_velocities_module - velocities_module) / velocities_module).T, label='error', color='green')
    ax2.legend()
    plt.legend()

    plt.show()
