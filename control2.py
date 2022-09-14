import sys
import time
import math
import numpy as np
from imu9_driver_v3 import Imu9IO
from tc74_driver_v2 import TempTC74IO
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO
from gps_driver_v2 import GpsIO

data_keys = ['time', 'd_phi', 'rpm_l', 'rpm_r', 'rpm_lb', 'rpm_rb', 'th_l', 'th_r']


def data_to_str(data):
    str_inf = ""
    for k, v in zip(data_keys, data):
        str_inf += k + ': ' + str(int(v)) + ' ; '
    return str_inf[:-3] + '\n'


def cap_to_phi(cap):
    if cap == 'S':
        return np.pi
    elif cap == 'W':
        return np.pi / 2
    elif cap == 'E':
        return -np.pi / 2
    else:
        return 0.0


def delta_odo(odo1, odo0):
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


class Control:
    def __init__(self, dt=0.5):
        self.ard = ArduinoIO()
        self.enc = EncoderIO()
        self.imu = Imu9IO()
        self.tpr = TempTC74IO()
        self.gps = GpsIO()

        self.dt = dt
        # set delay between old and new measures : HERE=dt
        self.enc.set_older_value_delay_v2(int(dt * 10))
        self.tpr.set_config(0x0, 0x0)
        self.tpr.set_mode(standby=True, side="both")

        self.cst = {'left': {'kp': 0.01, 'ki': 0.01},
                    'right': {'kp': 0.01, 'ki': 0.01},
                    'phi': {'kp': (3/4) / np.pi, 'ki': 8e-3 / np.pi}
                    'ligne': {'kv': 1, 'kn': 1}}

        self.positions = positions

        self.step_max = 50
        self.u_max = 100
        self.rpm_max = 4000

        self.ei_left, self.ei_right, self.ei_phi = 0, 0, 0
        self.cmd_left, self.cmd_right = 50, 50

    def reset(self, cmd_left_init=50, cmd_right_init=50):
        self.ei_left, self.ei_right, self.ei_phi = 0, 0, 0
        self.cmd_left, self.cmd_right = cmd_left_init, cmd_right_init

    def change_timing(self, dt):
        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * 10))

    def get_current_cap(self):
        return self.imu.orientation()

    def get_current_cap_degree(self):
        return self.get_current_cap() * (180 / np.pi)

    def get_rpm(self):
        # 1 : new ; 0 : old
        st1, st0 = self.enc.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(",")).astype(np.float)
        data_encoders1 = np.array(st1.split(",")).astype(np.float)

        odo_left0 = data_encoders0[4]
        odo_right0 = data_encoders0[3]

        odo_left1 = data_encoders1[4]
        odo_right1 = data_encoders1[3]

        rpm_left = (60. / 8.) * delta_odo(odo_left1, odo_left0) / self.dt
        rpm_right = (60. / 8.) * delta_odo(odo_right1, odo_right0) / self.dt

        return rpm_left, rpm_right

    def regulation_rpm(self, rpm_left_bar, rpm_right_bar):
        rpm_left, rpm_right = self.get_rpm()

        # left motor
        e_left = rpm_left_bar - rpm_left
        self.ei_left += e_left * self.dt
        step_left = self.cst['left']['kp'] * e_left + self.cst['left']['ki'] * self.ei_left

        # right motor
        e_right = rpm_right_bar - (-rpm_right)
        self.ei_right += e_right * self.dt
        step_right = self.cst['right']['kp'] * e_right + self.cst['right']['ki'] * self.ei_right

        # On seuil la variation en tension
        if abs(step_left) > self.step_max:
            step_left = self.step_max * step_left / abs(step_left)
        if abs(step_right) > self.step_max:
            step_right = self.step_max * step_right / abs(step_right)

        self.cmd_left = max(min(self.u_max, self.cmd_left + step_left), 0)
        self.cmd_right = max(min(self.u_max, self.cmd_right + step_right), 0)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

        # print('MEASURED RPM:', rpm_left, rpm_right)
        return rpm_left, rpm_right

    def leo_cap_and_speed(self, delta_phi, rpm_max):
        self.ei_phi += delta_phi * self.dt
        e_phi = self.cst['phi']['kp'] * delta_phi + self.cst['phi']['ki'] * self.ei_phi

        print("e_phi components: ", self.cst['phi']['kp'] * delta_phi, self.cst['phi']['ki'] * self.ei_phi)

        if e_phi >= 0:
            rpm_left_bar = rpm_max - e_phi * rpm_max
            rpm_right_bar = rpm_max
        else:
            rpm_right_bar = rpm_max + e_phi * rpm_max
            rpm_left_bar = rpm_max

        # print('RPM BAR:', rpm_left_bar, rpm_right_bar)
        return rpm_left_bar, rpm_right_bar

    def convert(self, data):
        lx_raw = data[0]
        ly_raw = data[2]
        lx = lx_raw // 100
        ly = ly_raw // 100
        lx += (lx_raw % 100) / 60
        ly += (ly_raw % 100) / 60
        if data[3] == 'W':
            ly = -ly
        return lx, ly

    def get_pos(self, coords, r):
        lx, ly = coords
        lx_ponton, ly_ponton = self.positions['ponton']
        x = r * math.cos(ly) * (lx - lx_ponton)
        y = r * (ly - ly_ponton)
        return x, y

    def get_dir(self, pos_a, pos_b, pos_boat):
        v = np.array([pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]])
        theta = math.atan2(v[1], v[0])
        n = np.array([-math.sin(theta), math.cos(theta)])
        n = n / np.linalg.norm(n)
        delta_p = np.array([pos_boat[0] - pos_a[0], pos_boat[1] - pos_a[1]])
        kv, kn = self.cst['ligne']['kv'], self.cst['ligne']['kn']
        dir = kv * v - kn * 2 * n @ n.T * (delta_p)
        d_x, d_y = dir
        psi_bar = math.atan2(d_y, d_x)
        return psi_bar

    def get_psi_bar_gps(self, data):
        lx, ly = self.convert(data)
        print(lx, "     ", ly)
        r = data[4]
        a = self.get_pos(self.positions['ponton'], r)
        b = self.get_pos(self.positions['nord'], r)
        pos_boat = self.get_pos(lx, ly, r)
        psi_bar = self.get_dir(a, b, pos_boat)
        return psi_bar

    def suivi_cap(self, duration, cap='N', speed_rpm=3000, mission_name='log'):
        file = open(mission_name + '.txt', 'a')
        file.write('duration: ' + str(duration) + ' ; ' + 'cap: ' + cap + ' ; spd: ' + str(speed_rpm) + '\n')

        self.reset()
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            phi = self.get_current_cap()
            delta_phi = sawtooth(cap_to_phi(cap) - phi)
            print("DELTA PHI: ", int(delta_phi * (180 / np.pi)))

            rpm_left_bar, rpm_right_bar = self.leo_cap_and_speed(delta_phi, speed_rpm)
            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, delta_phi * (180 / np.pi), rpm_left, rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            information = data_to_str(data)
            file.write(information)

            while (time.time() - t0loop) < self.dt:
                time.sleep(0.001)

        self.ard.send_arduino_cmd_motor(0, 0)
        file.close()

    def suivi_ligne(self, duration_max, a, b, speed_rpm=3000, mission_name='log'):
        file = open(mission_name + '.txt', 'a')
        file.write('duration: ' + str(duration_max) + ' ; ' + 'cap: ' + cap + ' ; spd: ' + str(speed_rpm) + '\n')

        self.reset()
        t0 = time.time()
        while (time.time() - t0) < duration_max:
            t0loop = time.time()

            msg, data = self.gps.read_gll_non_blocking()
            if msg:
                psi_bar = self.get_psi_bar_gps(data)
            else:
                pass
            phi = self.get_current_cap()
            delta_phi = sawtooth(psi_bar - phi)
            print("DELTA PHI: ", int(delta_phi * (180 / np.pi)))

            rpm_left_bar, rpm_right_bar = self.leo_cap_and_speed(delta_phi, speed_rpm)
            rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, delta_phi * (180 / np.pi), rpm_left, rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            information = data_to_str(data)
            file.write(information)

            while (time.time() - t0loop) < self.dt:
                time.sleep(0.001)

        self.ard.send_arduino_cmd_motor(0, 0)
        file.close()

if __name__ == '__main__':
    positions = {'ponton': [48.199024, -3.014790], 'nord': [48.199817, -3.015603], 'ouest': [48.199038, -3.015807]}

    try:
        start = str(sys.argv[0])
    except:
        start = 'ponton'

    try:
        finish = str(sys.argv[1])
    except:
        finish = 'ouest'

    # Chose a duration
    try:
        d = int(sys.argv[2])
    except:
        d = 60

    # Chose a speed
    try:
        s = int(sys.argv[3])
    except:
        s = 3000

    ctr = Control()

    ctr.suivi_ligne(d, positions[start], positions[finish], s)

