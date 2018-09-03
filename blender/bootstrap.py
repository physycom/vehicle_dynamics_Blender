"""
Auto-installation of dependencies that blender-addon needs

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

import os
import sys
import urllib.request
import subprocess
import importlib
from pathlib import Path

addon_path = str(Path(__file__).parent.parent)
blender_path = str(Path(sys.executable).parent)
# TODO detect version, make more flexible
# _, dirs, _ = next(os.walk(blender_path))
# blender_version_dir = dirs[0]
blender_version_dir = "2.79"
blender_python_dir = os.path.join(blender_path, blender_version_dir, "python")
posix_pip_location = os.path.join(blender_python_dir, "bin", "pip")
windows_pip_location = os.path.join(blender_python_dir, "scripts", "pip3.exe")
pip_location = {
    'posix':posix_pip_location,
    'windows':windows_pip_location
}
requirements_file_position = os.path.join(addon_path, "requirements.txt")


def call_system_command(command):
    try:
        retcode = subprocess.call(command, shell=True)
        if retcode < 0:
            print("Child was terminated by signal", -retcode, file=sys.stderr)
        else:
            return retcode
    except OSError as e:
        print("Execution failed:", e, file=sys.stderr)

def install_packages_from_requirements_file():
    command = None
    if (os.path.exists(pip_location['posix'])):
        command = r'"{}" install -r "{}"'.format(pip_location['posix'], requirements_file_position)
    elif os.path.exists(pip_location['windows']):
        command = r'"{}" install -r "{}"'.format(pip_location['windows'], requirements_file_position)
    if command:
        ret_code = call_system_command(command)
        if ret_code>0:
            raise Exception("Error installing dependencies")
    else:
        raise Exception("Error on finding pip location")

def uninstall_packages_from_requirements_file():
    command = None
    if (os.path.exists(pip_location['posix'])):
        command = r'"{}" uninstall -y -r "{}"'.format(pip_location['posix'], requirements_file_position)
    elif os.path.exists(pip_location['windows']):
        command = r'"{}" uninstall -y -r "{}"'.format(pip_location['windows'], requirements_file_position)
    if command:
        ret_code =  call_system_command(command)
        if ret_code>0:
            raise Exception("Error uninstalling dependencies")
    else:
        raise Exception("Error on finding pip location")


def check_modules_existence():
    # made a list instead of using requirements.txt because packages and modules name can differ
    # also reading the output of pip freeze in blender is tricky because it's open me another instance of blender
    required_modules = ['numpy','numba','quaternion']
    there_is_a_missing_package = False
    for required_module in required_modules:
        if not importlib.util.find_spec(required_module):
            there_is_a_missing_package = True
    if there_is_a_missing_package:
        return install_packages_from_requirements_file()
    else:
        return 0

def install_dependencies():
    # TODO handle permission errors

    print("Addon path " + addon_path)

    print("Blender path " + blender_path)

    if not (os.path.exists(posix_pip_location) or os.path.exists(windows_pip_location)):
        print("Downloading pip")
        # download get pip
        pip_download_location = os.path.join(addon_path, "get_pip.py")
        urllib.request.urlretrieve("https://bootstrap.pypa.io/get-pip.py",
                                   filename=pip_download_location)
        python_bin = os.path.join(blender_python_dir, "bin")
        if (os.path.exists(os.path.join(python_bin, "python3.5m"))):
            python_interpreter = os.path.join(python_bin, "python3.5m")
        elif os.path.exists(os.path.join(python_bin, "python.exe")):
            python_interpreter = os.path.join(python_bin, "python.exe")
        command = r'"{}" "{}"'.format(python_interpreter, pip_download_location)
        print("Command: " + command)
        call_system_command(command)
    return_code = check_modules_existence()
    return return_code
