"""
Provide a class to generate a trajectory and get accelerations from it.
Then check a integrated trajectory against original one to measure level of error.

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

import numpy as np
import quaternion

class TrajectoryGenerator:

    def __init__(self):
        self.x0 = 0  # initial x position
        self.v0x = 0  # initial x velocity
        self.ax = 0.1  # x acceleration
        self.wz = 0.1  # z angular velocity
        dt = 1e-2  # timestep
        self.r0 = np.array([0, 1, 0])  #  point position wrt cm
        self.t = np.arange(0, 100, dt)  # timestamps

        def rcm(t):
            """position at time t

            linear uniform accelerated motion along z
            """
            return np.array([
                0,
                0,
                self.x0 + self.v0x * t + 1 / 2 * self.ax * t ** 2
            ])

        def thetacm(t):
            """Angular position at time t of rotation around z"""
            return np.array([
                0,
                0,
                self.wz * t
            ])

        # position in all times of linear uniform accelerated motion along z
        r = np.array([rcm(ti) for ti in self.t]).T
        # angular position in all times
        th = np.array([thetacm(ti) for ti in self.t])
        # array of pure quaternion the angular positions
        thq = np.array([np.exp(quaternion.quaternion(*-thetai) / 2) for thetai in th])
        # get successive rotations of inital point r0
        r1 = np.array([(tq * quaternion.quaternion(*self.r0) * ~tq).components[1:4] for tq in thq])

        # add vertical offset to positions
        self.rp = r + r1.T