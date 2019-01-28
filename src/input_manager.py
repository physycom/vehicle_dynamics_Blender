"""
Utility module to parse input. It's auto-detects input type and returns parsed numpy array accordingly.

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

from enum import Enum, unique
from io import StringIO

import numpy as np
import pandas as pd


@unique
class InputType(Enum):
    ACCELERATION = 1
    FULLINERTIAL = 2
    GNSS = 3
    GYROSCOPE = 4
    INERTIAL = 5
    UNMOD_FULLINERTIAL = 6
    UNMOD_INERTIAL = 7
    UNRECOGNIZED = 8


def detect_input_type(df, filepath):
    """

    :param df:
    :param filepath:
    :return:
    """
    # default file type
    filetype = InputType.UNRECOGNIZED
    # analyze file name
    if filepath.find("acc") != -1:
        filetype = InputType.ACCELERATION
    elif filepath.find("gnss") != -1:
        filetype = InputType.GNSS
    elif filepath.find("gyr") != -1:
        filetype = InputType.GYROSCOPE
    elif filepath.find("unmodified-fullinertial") != -1:
        filetype = InputType.UNMOD_FULLINERTIAL
    elif filepath.find("unmodified-inertial") != -1:
        filetype = InputType.UNMOD_INERTIAL
    elif filepath.find("fullinertial") != -1:
        filetype = InputType.FULLINERTIAL
    elif filepath.find("inertial") != -1:
        filetype = InputType.INERTIAL
    # if no match are found try with column count
    else:
        column_count = df.shape[1]
        if column_count == 4:
            # TODO acc or gyr
            filetype = InputType.ACCELERATION
        elif column_count == 10:
            if df.isnull().values.any():
                filetype = InputType.UNMOD_INERTIAL
            else:
                filetype = InputType.INERTIAL
        elif column_count == 14:
            if df.isnull().values.any():
                filetype = InputType.UNMOD_FULLINERTIAL
            else:
                filetype = InputType.FULLINERTIAL
    return filetype


def get_vectors(df, input_type):
    """
    Get various numpy vectors based from dataframe columns based on input type

    :param df: pandas dataframe
    :param input_type: InputType enum
    :return:
        times, gps_speed, accelerations, angular_velocities if input is inertial \n
        times, coordinates, altitudes, gps_speed, heading, accelerations, angular_velocities if input is fullinertial
    """
    if input_type == InputType.INERTIAL or input_type == InputType.UNMOD_INERTIAL:
        accelerations = df[['ax', 'ay', 'az']].values.T
        angular_velocities = df[['gx', 'gy', 'gz']].values.T
        times = df['timestamp'].values.T
        gps_speed = df['speed'].values.T
        return times, gps_speed, accelerations, angular_velocities
    elif input_type == InputType.FULLINERTIAL:
        coordinates = df[['lat', 'lon']].values.T
        altitudes = df['alt'].values.T
        accelerations = df[['ax', 'ay', 'az']].values.T
        angular_velocities = df[['gx', 'gy', 'gz']].values.T
        times = df['timestamp'].values.T
        gps_speed = df['speed'].values.T
        heading = df['heading'].values.T
        return times, coordinates, altitudes, gps_speed, heading, accelerations, angular_velocities
    elif input_type == InputType.UNMOD_FULLINERTIAL:
        # TODO use DataFrame.interpolate
        # the input has gnss and inertial records mixed
        # filter gnss records
        clean_gnss_data = df.dropna(subset=['lat'])
        # interpolate coordinates
        from numpy import interp
        gnss_data = clean_gnss_data[['lat', 'lon', 'alt', 'heading', 'speed']].values
        gnss_data_timestamp = clean_gnss_data['timestamp'].values
        # filter inertial records from dataframe
        df = df.dropna(subset=['ax'])
        accelerations = df[['ax', 'ay', 'az']].values.T
        angular_velocities = df[['gx', 'gy', 'gz']].values.T
        times = df['timestamp'].values.T
        try:
            coordinatesX = interp(times, gnss_data_timestamp, gnss_data[:,0])
            coordinatesY = interp(times,gnss_data_timestamp,gnss_data[:,1])
            coordinates = np.array([x for x in zip(coordinatesX, coordinatesY)]).T
            altitudes = interp(times,gnss_data_timestamp,gnss_data[:,2])
            heading = interp(times,gnss_data_timestamp,gnss_data[:,3])
            gps_speed = interp(times,gnss_data_timestamp,gnss_data[:,4])
        except (ValueError):
            print("raised exception on interpolation, this can be caused by records with same timestamp")
            # if an exception is raised it can be caused by x vector not being sorted, so sort it
            # this is a rare case and is sign of a bad input dataset
            # TODO handle case
        # correct heading
        heading = 270 - heading
        return times, coordinates, altitudes, gps_speed, heading, accelerations, angular_velocities


def parse_input(filepath, accepted_types=[input_type for input_type in InputType], slice_start=None, slice_end=None):
    """ Parse input file from filetype

    If the input is not one of the specified accepted format raise Expection
    Additional slicing can be specified with sliceStar and sliceEnd

    :param filepath: string
    :param accepted_types: list of accepted input types from <InputType> enum. Default accept all types.
    :param slice_start: integer
    :param slice_end: integer
    :return:
        times, gps_speed, accelerations, angular_velocities if input is inertial \n
        times, coordinates, gps_speed, heading, accelerations, angular_velocities if input is fullinertial
    :raises:
        Exception if format is not accepted or recognized
    """

    # open file
    with open(filepath, mode='r') as file:
        # read all content into string
        file_content = file.read()
    # remove beginning hashtag and tab
    file_content = file_content.strip("#\t")
    # i used this technique to not modify input file
    # use StringIO to simulate file read
    string_io = StringIO(file_content)
    # use pandas to parse tsv
    # set to not use first column as index
    df = pd.read_csv(string_io, sep='\t',index_col=False)
    # detect file type
    input_type = detect_input_type(df, filepath)

    if input_type != InputType.UNRECOGNIZED:
        # slice input
        if slice_start is not None and slice_start < 0:
            raise Exception("Slice start must be positive")
        if slice_end is not None and abs(slice_end) > df.shape[0]:
            raise Exception("Slice end must not exceed dataframe rows")
        df = df[slice_start:slice_end]
        if input_type not in accepted_types:
            raise Exception("Not accepted format")
        # extrapolate vectors from input
        return get_vectors(df, input_type)
    else:
        raise Exception("Unrecognized input format")
