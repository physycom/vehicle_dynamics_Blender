import numpy as np

from clean_data_utils import converts_measurement_units, reduce_disturbance, \
    clear_gyro_drift, correct_z_orientation, normalize_timestamp, \
    sign_inversion_is_necessary, get_stationary_times, correct_xy_orientation
from gnss_utils import get_positions, get_velocities, align_to_world, get_accelerations
from input_manager import parse_input, InputType
from integrate import rotate_accelerations, simps_integrate


def get_positions_times(path):
    window_size = 20

    times, coordinates, altitudes, gps_speed, accelerations, angular_velocities = parse_input(path, [
        InputType.UNMOD_FULLINERTIAL])
    converts_measurement_units(accelerations, angular_velocities, gps_speed, coordinates)

    # GNSS data handling
    gnss_positions, headings = get_positions(coordinates, altitudes)

    # reduce accelerations disturbance
    times, accelerations = reduce_disturbance(times, accelerations, window_size)
    # reduce angular velocities disturbance
    _, angular_velocities = reduce_disturbance(times, angular_velocities, window_size)
    # truncate others array to match length of times array
    gnss_positions = gnss_positions[:, round(window_size / 2):-round(window_size / 2)]

    real_velocities = get_velocities(times, gnss_positions)
    real_acc = get_accelerations(times, real_velocities)
    real_speeds = np.linalg.norm(real_velocities, axis=0)

    stationary_times = get_stationary_times(real_speeds)

    angular_velocities = clear_gyro_drift(angular_velocities, stationary_times)
    normalize_timestamp(times)

    accelerations, angular_velocities = correct_z_orientation(accelerations, angular_velocities, stationary_times)

    # remove g
    accelerations[2] -= accelerations[2, stationary_times[0][0]:stationary_times[0][-1]].mean()

    accelerations = correct_xy_orientation(accelerations, angular_velocities)

    # convert to laboratory frame of reference
    accelerations = rotate_accelerations(times, accelerations, angular_velocities)
    accelerations = align_to_world(gnss_positions, accelerations, stationary_times)

    initial_speed = np.array([[gps_speed[0]], [0], [0]])
    correct_velocities = simps_integrate(times, accelerations, initial_speed, adjust_data=real_velocities,
                                         adjust_frequency=1)

    if sign_inversion_is_necessary(correct_velocities):
        accelerations *= -1
        correct_velocities *= -1

    correct_position = simps_integrate(times, correct_velocities, adjust_data=gnss_positions, adjust_frequency=1)

    return correct_position, times


if __name__ == '__main__':
    # for benchmarking
    import time, sys, argparse, os

    parser = argparse.ArgumentParser(description='Inertia[+GNSS] data to trajectory')
    parser.add_argument('input', type=str, help='Input file')
    parser.add_argument('output', type=str, help='Output file')
    args = parser.parse_args()
    start_time = time.time()

    my_path = os.path.abspath(os.path.dirname(__file__))
    path = os.path.join(my_path, args.input)

    positions, times = get_positions_times(path)

    times = np.reshape(times, (1, len(times)))
    timesPosition = np.concatenate((times, positions), axis=0)
    np.savetxt(args.output, timesPosition.T, delimiter=";", newline="\n")

    message = "Execution time: {} seconds | dataset size {}".format(time.time() - start_time, positions.shape[1])
    print(message)
