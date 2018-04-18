#!/usr/bin/env python
"""
Plot integrated velocities and trajectory of data from test_fixtures/raw_inertial_data.txt

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
from scipy.linalg import norm

from plots_scripts.plot_utils import plot_vectors
from src.clean_data_utils import converts_measurement_units, reduce_disturbance, \
    clear_gyro_drift, correct_z_orientation, normalize_timestamp, \
    sign_inversion_is_necessary, get_stationary_times, parse_input as correct_parse_input
from src.input_manager import parse_input, InputType
from src.integrate import rotate_accelerations, simps_integrate
from src.gnss_utils import get_positions, get_speeds, align_to_world

if __name__ == '__main__':

    # for benchmarking
    import time

    start_time = time.time()

    parking_fullinertial_unmod = 'tests/test_fixtures/parking.tsv'
    times, coordinates, altitudes, gps_speed , accelerations, angular_velocities = parse_input(parking_fullinertial_unmod, [InputType.UNMOD_FULLINERTIAL])
    converts_measurement_units(accelerations, angular_velocities, gps_speed, coordinates)
    gnss_positions = get_positions(coordinates, altitudes)
    real_speed = get_speeds(times, gnss_positions)
    stationary_times = get_stationary_times(real_speed)
    # reduce accelerations disturbance
    times, accelerations = reduce_disturbance(times, accelerations)
    # reduce angular velocities disturbance
    _, angular_velocities = reduce_disturbance(times, angular_velocities)
    angular_velocities = clear_gyro_drift(angular_velocities, stationary_times)
    normalize_timestamp(times)
    accelerations, angular_velocities = correct_z_orientation(accelerations, angular_velocities, stationary_times)
    # remove g
    accelerations[2] -= accelerations[2, stationary_times[0][0]:stationary_times[0][-1]].mean()
    # convert to laboratory frame of reference
    accelerations = rotate_accelerations(times, accelerations, angular_velocities)
    accelerations = align_to_world(gnss_positions,accelerations,stationary_times)
    velocities = simps_integrate(times, accelerations)

    if sign_inversion_is_necessary(velocities):
        accelerations *= -1
        velocities *= -1

    positions = simps_integrate(times, velocities)

    print("Execution time: %s seconds" % (time.time() - start_time))

    # plotting
    velocities_norm = norm(velocities, axis=0)
    plt.plot(real_speed, label='gnss speed')
    plt.plot(velocities_norm, label='integrated velocities norm')
    plt.legend()

    figure = plot_vectors(gnss_positions, "positions")

    lim3d = figure.axes[1].get_xbound()
    figure.axes[1].set_zbound(lim3d)
    figure.axes[1].set_ybound(lim3d)

    plt.show()
