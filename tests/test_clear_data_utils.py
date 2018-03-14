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

from src.clean_data_utils import *
import pandas as pd

# number of second of
car_initial_stationary_time = 40
drift_tolerance = 0.01


class ClearDataUtilsTest(unittest.TestCase):

    def setUp(self):
        self.df = pd.read_csv('test_fixtures/raw_inertial_data.txt', sep='\t')

    def test_clearGyroDrift(self):
        # get initial stationary time angular speed around x-axis
        initial_stationary_time_gx_value = self.df.loc[0:car_initial_stationary_time, 'gx'].mean()
        # check there is a gyroscope drift
        assert abs(initial_stationary_time_gx_value) > drift_tolerance
        # call util to remove drift
        clear_gyro_drift(self.df)
        # re-calculate initial angular speed around x-axis
        initial_stationary_time_gx_value = self.df.loc[0:car_initial_stationary_time, 'gx'].mean()
        # check that the drift is lower than a tolerance
        assert abs(initial_stationary_time_gx_value) < drift_tolerance

    def test_converts_measurement_units(self):
        df = pd.DataFrame.from_dict({'ax': [1], 'ay': [1], 'az': [1], 'gx': [180], 'gy': [180], 'gz': [180]})
        converts_measurement_units(df)
        from scipy.constants import g, pi
        assert df.at[0, 'ax'] == g
        assert df.at[0, 'gx'] == pi

    def test_normalize_timestamp(self):
        assert self.df.at[0, 'timestamp'] != 0
        normalize_timestamp(self.df)
        assert self.df.at[0, 'timestamp'] == 0

    def test_reduce_disturbance(self):
        variance_reduction_factor = 500
        # get variance before reduction
        variance_before = self.df['gx'].var()
        # reduce disturbance
        reduce_disturbance(self.df)
        # check that the variance has been reduced by a factor
        variance_after = self.df['gx'].var()
        ratio = variance_before / variance_after
        assert ratio >= variance_reduction_factor

    def test_correct_z_orientation(self):
        threshold = 0.1
        columns = {value: position for position, value in enumerate(self.df.columns.values)}
        values = correct_z_orientation(self.df.values, columns)
        self.df = pd.DataFrame(columns=self.df.columns, data=values)
        stationary_ax_mean = self.df.loc[0:40, 'ax'].mean()
        stationary_ay_mean = self.df.loc[0:40, 'ay'].mean()
        stationary_az_mean = self.df.loc[0:40, 'az'].mean()
        assert abs(stationary_ax_mean) < threshold
        assert abs(stationary_ay_mean) < threshold
        assert abs(stationary_az_mean - 1) < threshold
