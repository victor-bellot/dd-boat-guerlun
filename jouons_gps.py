import sys
import time
import math
import numpy as np
from imu9_driver_v3 import Imu9IO
from tc74_driver_v2 import TempTC74IO
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO
from gps_driver_v2 import GpsIO


def get_pos(lx, ly, r):
    x = r * math.cos(ly) * (lx - lx_hat)
    y = r * (ly - ly_hat)
    return x, y

def cap_gps(pos_a, pos_b, pos_boat):
    v = np.array([pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]])
    theta = math.atan2(v[1], v[0])
    n = np.array([-math.sin(theta), math.cos(theta)])
    n = n / np.linalg.norm(n)
    delta_p = np.array([pos_boat[0] - pos_a[0], pos_boat[1] - pos_a[1]])
    dir = v - 2 * n @ n.T * (delta_p)
    d_x,  d_y = dir
    phi_bar = math.atan2(d_y, d_x)
    return phi_bar

if __name__ == '__main__':
    gps = GpsIO()
    lx_hat, ly_hat = 3.014790, 48.199024

    while True:
        data = gps.read_gll_non_blocking()
        if data[0]:
            data = data[1]
            data[0] = data[0] / 100
            data[2] = data[2] / 100
            print(data)
            r = data[4]
            a = get_pos(lx_hat, ly_hat, r)
            b = get_pos(lx_hat, 48.199200, r)
            pos_boat = get_pos(data[2], data[0], r)
            print(cap_gps(a, b, pos_boat))
            time.sleep(0.1)
        else:
            pass
