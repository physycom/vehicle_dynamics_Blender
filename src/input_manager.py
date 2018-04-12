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


from io import StringIO
from enum import Enum, auto, unique
import pandas as pd

@unique
class InputType(Enum):
    ACCELERATION = auto()
    FULLINERTIAL = auto()
    GNSS = auto()
    GYROSCOPE = auto()
    INERTIAL = auto()
    UNMOD_FULLINERTIAL = auto()
    UNMOD_INERTIAL = auto()
    UNRECOGNIZED = auto()


def detect_input_type(df,filepath):
    """

    :param df:
    :param filepath:
    :return:
    """
    # default file type
    filetype = InputType.UNRECOGNIZED
    # analyze file name
    if filepath.find("acc")!=-1:
        filetype = InputType.ACCELERATION
    elif filepath.find("gnss")!=-1:
        filetype = InputType.GNSS
    elif filepath.find("gyr")!=-1:
        filetype = InputType.GYROSCOPE
    elif filepath.find("unmodified-fullinertial")!=-1:
        filetype = InputType.UNMOD_FULLINERTIAL
    elif filepath.find("unmodified-inertial")!=-1:
        filetype = InputType.UNMOD_INERTIAL
    elif filepath.find("fullinertial")!=-1:
        filetype = InputType.FULLINERTIAL
    elif filepath.find("inertial")!=-1:
        filetype = InputType.INERTIAL
    # if no match are found try with column count
    else:
        column_count = df.shape[1]
        if column_count == 4:
            # TODO acc or gyr
            filetype = InputType.ACCELERATION
        elif column_count==10:
            # TODO mod / unmod
            filetype = InputType.INERTIAL
        elif column_count==14:
            # TODO mod / unmod
            filetype = InputType.UNMOD_FULLINERTIAL
    return filetype

def get_vectors(df,input_type):
    if input_type == InputType.INERTIAL or input_type == InputType.UNMOD_INERTIAL:
        accelerations = df[['ax', 'ay', 'az']].values.T
        angular_velocities = df[['gx', 'gy', 'gz']].values.T
        times = df['timestamp'].values.T
        gps_speed = df['speed'].values.T
        return times, gps_speed, accelerations, angular_velocities
    elif input_type == InputType.FULLINERTIAL or input_type == InputType.UNMOD_FULLINERTIAL:
        coordinates = df[['lat','lon']].values.T
        accelerations = df[['ax', 'ay', 'az']].values.T
        angular_velocities = df[['gx', 'gy', 'gz']].values.T
        times = df['timestamp'].values.T
        gps_speed = df['speed'].values.T
        return times, coordinates, gps_speed, accelerations, angular_velocities



def parse_input(filepath,accepted_types=[type for type in InputType],sliceStart=None,sliceEnd=None):
    """ Parse input file from filetype

    If the input is not one of the specified accepted format raise Expection
    Additional slicing can be specified with sliceStar and sliceEnd

    :param filepath: string
    :param accepted_types: list of accepted input types from <InputType> enum. Default accept all types.
    :param sliceStart: integer
    :param sliceEnd: integer
    """

    # open file
    with open(filepath,mode='r') as file:
        # read all content into string
        file_content = file.read()
    # remove beginning hashtag and tab
    file_content = file_content.strip("#\t")
    # i used this technique to not modify input file
    # use StringIO to simulate file read
    stringIO = StringIO(file_content)
    # use pandas to parse tsv
    df = pd.read_csv(stringIO,sep='\t')
    # detect file type
    input_type = detect_input_type(df,filepath)

    if (input_type!=InputType.UNRECOGNIZED):
        # slice input
        df = df[sliceStart:sliceEnd]

        if (input_type not in accepted_types):
            raise Exception("Not accepted format")
        # extrapolate vectors from input
        return get_vectors(df,input_type)
    else:
        raise Exception("Unrecognized input format")
