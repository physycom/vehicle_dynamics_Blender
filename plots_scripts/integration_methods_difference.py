#!/usr/bin/env python
"""
Plot average error of quad, trapezoid and simpson integration method
on TrajectoryGenerator motion

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

from tests.test_integration import integrate_and_test, Trajectory, \
    quad_integrate, trapz_integrate, simps_integrate

if __name__ == 'main':
    trajectory = Trajectory()
    quad_error = integrate_and_test(quad_integrate)
    trapz_error = integrate_and_test(trapz_integrate)
    simps_error = integrate_and_test(simps_integrate)
    plt.plot(quad_error, label='quad')
    plt.plot(trapz_error, label='trapz')
    plt.plot(simps_error, label='simps')
    plt.legend()
    plt.show()
