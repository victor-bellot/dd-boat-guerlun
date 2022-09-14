import sys
import time
import math
import numpy as np
#from imu9_driver_v3 import Imu9IO
#from tc74_driver_v2 import TempTC74IO
#from arduino_driver_v2 import ArduinoIO
#from encoders_driver_v2 import EncoderIO
from gps_driver_v2 import GpsIO

def convert(data):
    lx_raw = data[0]
    ly_raw = data[2]
    lx = lx_raw // 100
    ly = ly_raw // 100
    lx += (lx_raw % 100) / 60
    ly += (ly_raw % 100) / 60
    if data[3] == 'W':
        ly = -ly
    return lx, ly

def get_pos(coords, r):
    lx, ly = coords
    lx_ponton, ly_ponton = positions['ponton']
    x = r * math.cos(ly) * (lx - lx_ponton)
    y = r * (ly - ly_ponton)
    return x, y

def cap_gps(pos_a, pos_b, pos_boat):
    v = np.array([pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]])
    theta = math.atan2(v[1], v[0])
    n = np.array([-math.sin(theta), math.cos(theta)])
    n = n / np.linalg.norm(n)
    delta_p = np.array([pos_boat[0] - pos_a[0], pos_boat[1] - pos_a[1]])
    k1, k2 = 0, 1
    dir = k1 * v - k2 * 2 * n @ n.T * (delta_p)
    d_x,  d_y = dir
    psi_bar = math.atan2(d_y, d_x)
    return psi_bar

def get_cap(data):
    lx, ly = convert(data)
    print(lx, "     ", ly)
    r = data[4]
    a = get_pos(positions['ponton'], r)
    b = get_pos(positions['nord'], r)
    pos_boat = get_pos(lx, ly, r)
    psi_bar = cap_gps(a, b, pos_boat)
    return psi_bar

if __name__ == '__main__':
    gps = GpsIO()
    positions = {'ponton': [48.199024, -3.014790], 'nord': [48.199817, -3.015603], 'ouest': [48.199038, -3.015807]}


    while True:
        msg, data = gps.read_gll_non_blocking()
        if msg:
            psi_bar = get_cap(data)
            print(psi_bar)
            time.sleep(0.1)
        else:
            pass

