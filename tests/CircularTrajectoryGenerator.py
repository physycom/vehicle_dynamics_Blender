"""
Circular trajectory on 2d plane with object rotation.
Useful for testing local to laboratory.

This file is part of inertial_to_blender project,
a Blender simulation generator from inertial sensor data on cars.

Copyright (C) 2018  Federico Bertani
Author: Federico Bertani, Alessandro Fabbri
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

from BaseTrajectoryGenerator import BaseTrajectoryGenerator


class CircularTrajectoryGenerator(BaseTrajectoryGenerator):

    def get_analytical_accelerations(self):
        pass

    def get_analytical_velocities(self):
        pass

    def get_numerical_derived_accelerations(self):
        pass

    def get_start_velocity(self):
        pass

    def check_trajectory(self, external_trajectory):
        pass

    def plot_trajectory(self):
        pass
