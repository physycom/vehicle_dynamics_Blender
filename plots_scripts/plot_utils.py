"""
Various plotting routines

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
# needed for 3d projection


def plot_vectors(vectors, label=None):
    """ Plot 3xn numpy array in 2d and 3d

    plt.show() blocking call must be called to show all plot created whit this function
    :param vectors: 3xn numpy array
    :param label: optional custom 3d plot label
    :return figure
    """

    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax = fig.add_subplot(1, 2, 1)
    [ax.plot(ri, label=str(i)) for i, ri in enumerate(vectors)]
    plt.legend()

    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.plot(*vectors, label=label)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.legend()

    return fig
