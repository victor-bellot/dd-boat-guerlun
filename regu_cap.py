from imu9_driver_v2 import *

imu = Imu9IO()
import numpy as np


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi  # or equivalently   2*arctan(tan(x/2))

    # rpm_right_bar=rpm-rpm_left_bar
    # rpm_right_bar-rpm_left_bar=K3*sawtooth(phi_bar-phi)


