"""
Shows gyroscope drift

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
import sys
import numpy as np

from src.clean_data_utils import converts_measurement_units, reduce_disturbance, normalize_timestamp, get_stationary_times, clear_gyro_drift
from src.input_manager import parse_input, InputType

if __name__ == "__main__":
    window_size = 20

    # currently default format is unmodified fullinertial but other formats are / will be supported
    times, coordinates, altitudes, gps_speed, heading, accelerations, angular_velocities = parse_input(sys.argv[1], [
        InputType.UNMOD_FULLINERTIAL])

    converts_measurement_units(accelerations, np.array([0.1]), gps_speed, coordinates, heading)

    normalize_timestamp(times)

    xlim= (20,40)
    ylim= (-1,2)

    fig = plt.figure()
    fig1_ax1 = fig.add_subplot(1, 1, 1)
    fig1_ax1.set_title("Before")
    fig1_ax1.set_xlabel("time (s)")
    fig1_ax1.set_ylabel("acceleration (m/s²)", color='b')
    fig1_ax1.tick_params("y", color='b')
    fig1_ax1.plot(times, accelerations[1],color='b')
    fig1_ax1.set_xlim(xlim)
    fig1_ax1.set_ylim(ylim)
    fig1_ax2 = fig1_ax1.twinx()
    fig1_ax2.set_ylabel("angular velocity (deg/s)", color='r')
    fig1_ax2.tick_params("y", colors='r')
    fig1_ax2.plot(times, -angular_velocities[2], color='r')
    fig1_ax2.set_ylim(ylim)
    fig1_ax2.set_xlim(xlim)
    plt.legend()

    # reduce accelerations disturbance
    times, accelerations = reduce_disturbance(times, accelerations, window_size)
    # reduce angular velocities disturbance
    _, angular_velocities = reduce_disturbance(times, angular_velocities, window_size)
    gps_speed = gps_speed[round(window_size / 2):-round(window_size / 2)]

    # get time windows where vehicle is stationary
    stationary_times = get_stationary_times(gps_speed)

    # clear gyroscope drift
    angular_velocities = clear_gyro_drift(angular_velocities, stationary_times)

    fig2 = plt.figure()
    fig2_ax1 = fig2.add_subplot(1, 1, 1)
    fig2_ax1.set_title("After")
    fig2_ax1.set_xlabel("time (s)")
    fig2_ax1.set_ylabel("acceleration (m/s²)", color='b')
    fig2_ax1.tick_params("y", color='b')
    fig2_ax1.plot(times, accelerations[1],color='b')
    fig2_ax1.set_ylim(ylim)
    fig2_ax1.set_xlim(xlim)
    fig2_ax2 = fig2_ax1.twinx()
    fig2_ax2.set_ylabel("angular velocity (deg/s)", color='r')
    fig2_ax2.tick_params("y", colors='r')
    fig2_ax2.plot(times, -angular_velocities[2], color='r')
    fig2_ax2.set_ylim(ylim)
    fig2_ax1.set_xlim(xlim)
    plt.legend()
    plt.show()
