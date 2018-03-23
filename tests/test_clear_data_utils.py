"""
Tests for clear_data_utils module

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

import unittest

import numpy as np
import pandas as pd

from src.clean_data_utils import reduce_disturbance, parse_input, normalize_timestamp, converts_measurement_units, \
    correct_z_orientation, clear_gyro_drift
from test_fixtures import car_initial_stationary_time

drift_tolerance = 0.01


class ClearDataUtilsTest(unittest.TestCase):

    def setUp(self):
        df = pd.read_csv('tests/test_fixtures/raw_inertial_data.txt', sep='\t')
        self.times, self.gps_speed, self.accelerations, self.angular_velocities = parse_input(df)

    def test_clearGyroDrift(self):
        _, self.angular_velocities = reduce_disturbance(self.times, self.angular_velocities)
        # get initial stationary time angular speed around x-axis
        initial_stationary_time_gx_value = self.angular_velocities[0, 0:car_initial_stationary_time].mean()
        # check there is a gyroscope drift
        assert abs(initial_stationary_time_gx_value) > drift_tolerance
        # call util to remove drift
        self.angular_velocities = clear_gyro_drift(self.angular_velocities)
        # re-calculate initial angular speed around x-axis
        initial_stationary_time_gx_value = self.angular_velocities[0, 0:car_initial_stationary_time].mean()
        # check that the drift is lower than a tolerance
        assert abs(initial_stationary_time_gx_value) < drift_tolerance

    def test_converts_measurement_units(self):
        # create temporary measures to convert
        accelerations = np.array([[1.0], [1.0], [1.0]])
        angular_velocities = np.array([[180.0], [180.0], [180.0]])
        gps_speed = np.array([[1.0]])
        converts_measurement_units(gps_speed, accelerations, angular_velocities)
        # import scipy constants
        from scipy.constants import g, pi, kmh
        # check measurement unit conversion
        self.assertTrue(all(accelerations == g))
        self.assertTrue(all(angular_velocities == pi))
        self.assertTrue(all(gps_speed == kmh))

    def test_normalize_timestamp(self):
        # check that the first timestamp if not zero
        self.assertTrue(self.times[0] != 0)
        # normalize timestamps
        normalize_timestamp(self.times)
        # check that now it's zero
        self.assertTrue(self.times[0] == 0)

    def test_reduce_disturbance(self):
        variance_reduction_factor = 10
        # get variance before reduction
        variance_before = self.angular_velocities.var(axis=1).T[0]
        # reduce disturbance
        _, self.angular_velocities = reduce_disturbance(self.times, self.angular_velocities)
        # check that the variance has been reduced by a factor
        variance_after = self.angular_velocities.var(axis=1).T[0]
        ratio = variance_before / variance_after
        self.assertTrue(ratio >= variance_reduction_factor)

    def test_correct_z_orientation(self):
        _, self.accelerations = reduce_disturbance(self.times, self.accelerations)
        threshold = 0.1
        # get average value in start stationary time
        stationary_ax_mean_before = self.accelerations[0, 0:car_initial_stationary_time].mean()
        stationary_ay_mean_before = self.accelerations[1, 0:car_initial_stationary_time].mean()
        # execute test only if there is a acceleration component on x/y when the car should be stationary
        if abs(stationary_ax_mean_before) > threshold or abs(stationary_ay_mean_before) > threshold:
            # correct z orientation
            self.accelerations, self.angular_velocities = correct_z_orientation(self.accelerations,
                                                                                self.angular_velocities)
            # get average value in start stationary time
            stationary_ax_mean_after = self.accelerations[0, 0:car_initial_stationary_time].mean()
            stationary_ay_mean_after = self.accelerations[1, 0:car_initial_stationary_time].mean()
            # in x and y axis it shouldn't be any acceleration
            assert stationary_ax_mean_after < stationary_ax_mean_before
            assert stationary_ay_mean_after < stationary_ay_mean_before
