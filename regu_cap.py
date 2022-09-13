from imu9_driver_v2 import *

imu = Imu9IO()
import numpy as np


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi  # or equivalently   2*arctan(tan(x/2))

    # rpm_right_bar=rpm-rpm_left_bar
    # rpm_right_bar-rpm_left_bar=K3*sawtooth(phi_bar-phi)

def suivi_cap(phi_bar, rpm_max=8000):
    K3 = rpm_max / (2 * np.pi)
    phi = imu.orientation() * (180 / np.pi)
    ec_ang = K3 * sawtooth(phi_bar - phi)
    if ec_angle >= 0:
        rpm_left_bar = rpm_max - ec_ang
        rpm_right_bar = rpm_max - rpm_right_bar
    else:
        rpm_right_bar = rpm_max - ec_ang
        rpm_left_bar = rpm_max - rpm_left_bar
    return rpm_left_bar, rpm_right_bar
