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
import numpy as np

from plots_scripts.plot_utils import plot_vectors
from src.clean_data_utils import converts_measurement_units, reduce_disturbance, \
    clear_gyro_drift, correct_z_orientation, normalize_timestamp, \
    sign_inversion_is_necessary, get_stationary_times, correct_xy_orientation
from src.gnss_utils import get_positions, get_velocities, get_accelerations, get_first_motion_time, \
    get_initial_angular_position
from src.input_manager import parse_input, InputType
from src.integrate import cumulative_integrate
from src import rotate_accelerations, align_to_world

if __name__ == '__main__':

    window_size = 20

    # for benchmarking
    import time

    start_time = time.time()

    parking_fullinertial_unmod = 'tests/test_fixtures/parking.tsv'
    times, coordinates, altitudes, gps_speed, heading, accelerations, angular_velocities = parse_input(parking_fullinertial_unmod, [InputType.UNMOD_FULLINERTIAL])
    converts_measurement_units(accelerations, angular_velocities, gps_speed, coordinates)

    # GNSS data handling
    gnss_positions, headings = get_positions(coordinates, altitudes)

    # reduce accelerations disturbance
    times, accelerations = reduce_disturbance(times, accelerations, window_size)
    # reduce angular velocities disturbance
    _, angular_velocities = reduce_disturbance(times, angular_velocities, window_size)
    # truncate others array to match length of times array
    gnss_positions = gnss_positions[:,round(window_size/2):-round(window_size/2)]

    real_velocities = get_velocities(times, gnss_positions)
    real_acc = get_accelerations(times, real_velocities)
    real_speeds = np.linalg.norm(real_velocities, axis=0)

    stationary_times = get_stationary_times(gps_speed)

    angular_velocities = clear_gyro_drift(angular_velocities, stationary_times)
    normalize_timestamp(times)

    accelerations, angular_velocities = correct_z_orientation(accelerations, angular_velocities, stationary_times)

    # remove g
    accelerations[2] -= accelerations[2, stationary_times[0][0]:stationary_times[0][-1]].mean()

    plot_vectors([accelerations[0:2], angular_velocities[2]],
                          ['inertial_ax', 'omega_z'],title="inertial accelerations before rotations",tri_dim=False)

    accelerations = correct_xy_orientation(accelerations,angular_velocities)

    plot_vectors([accelerations[0:2], angular_velocities[2]],
                          ['inertial_ax', 'omega_z'],title="inertial accelerations after rotations",tri_dim=False)

    # convert to laboratory frame of reference
    motion_time = get_first_motion_time(stationary_times, gnss_positions)
    initial_angular_position = get_initial_angular_position(gnss_positions, motion_time)

    # convert to laboratory frame of reference
    accelerations, angular_positions = rotate_accelerations(times, accelerations, angular_velocities, heading,
                                                            initial_angular_position)

    # rotate to align y to north, x to east
    accelerations = align_to_world(gnss_positions, accelerations, motion_time)

    figure = plot_vectors([accelerations[0:2],real_acc[0:2], angular_velocities[2]],
                          ['inertial_ax','gnss_acc', 'omega_z'],
                          title="comparison inertial and gnss accelerations in word reference frame",tri_dim=False)
    plt.show()

    initial_speed = np.array([[gps_speed[0]],[0],[0]])
    correct_velocities = cumulative_integrate(times, accelerations, initial_speed, adjust_data=real_velocities, adjust_frequency=1)

    correct_position = cumulative_integrate(times, correct_velocities, adjust_data=gnss_positions, adjust_frequency=1)

    print("Execution time: %s seconds" % (time.time() - start_time))

    # plotting

    figure = plot_vectors([correct_position,gnss_positions],['x integrated positions','x GNSS positions'])

    lim3d = figure.axes[1].get_xbound()
    figure.axes[1].set_zbound(lim3d)
    figure.axes[1].set_ybound(lim3d)
    plt.show()
