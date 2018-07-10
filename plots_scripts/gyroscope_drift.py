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

from src.clean_data_utils import converts_measurement_units, reduce_disturbance, normalize_timestamp
from src.input_manager import parse_input, InputType

if __name__ == "__main__":
    window_size = 20

    # currently default format is unmodified fullinertial but other formats are / will be supported
    times, coordinates, altitudes, gps_speed, heading, accelerations, angular_velocities = parse_input(sys.argv[1], [
        InputType.UNMOD_FULLINERTIAL])

    converts_measurement_units(accelerations, np.array([0.1]), gps_speed, coordinates, heading)

    # reduce accelerations disturbance
    times, accelerations = reduce_disturbance(times, accelerations, window_size)
    # reduce angular velocities disturbance
    _, angular_velocities = reduce_disturbance(times, angular_velocities, window_size)
    gps_speed = gps_speed[round(window_size / 2):-round(window_size / 2)]
    normalize_timestamp(times)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(times, gps_speed, label='speed m/s')
    ax.fill_between(x=times,y1=-angular_velocities[2].T,y2=0,label='omega_z deg/s', color='r', alpha=0.5)
    ax.set_xlabel("seconds")
    plt.legend()
    plt.show()
