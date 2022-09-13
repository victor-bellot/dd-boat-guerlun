import sys
import time
import math
import numpy as np
from imu9_driver_v2 import Imu9IO
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO


def delta_odo(odo1, odo0):
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def sawtooth(x):
    return (x+np.pi) % (2*np.pi) - np.pi


class Control:
    def __init__(self, dt=0.5, measurement_per_dt=10):
        self.ard = ArduinoIO()
        self.enc = EncoderIO()
        self.imu = Imu9IO()

        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * measurement_per_dt))

        self.cst = {'left': {'kp': 0.01, 'ki': 0.01},
                    'right': {'kp': 0.01, 'ki': 0.01},
                    'cap': 300}

        self.step_max = 50
        self.u_max = 150
        self.rpm_max = 10000

        self.ei_left, self.ei_right = 0, 0
        self.cmd_left, self.cmd_right = 50, 50

    def reset(self, cmd_left_init=50, cmd_right_init=50):
        self.ei_left, self.ei_right = 0, 0
        self.cmd_left, self.cmd_right = cmd_left_init, cmd_right_init

    def change_timing(self, dt, measurement_per_dt):
        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * measurement_per_dt))

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

        self.cmd_left = min(self.u_max, self.cmd_left + step_left)
        self.cmd_right = min(self.u_max, self.cmd_right + step_right)

        print(rpm_left, rpm_right, step_left, step_right, self.cmd_left, self.cmd_right)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

    def regulation_cap_and_speed(self, cap, speed_rpm):
        current_cap = self.get_current_cap()
        delta = self.cst['cap'] * sawtooth(cap - current_cap)
        print('DELTA CAP:', delta)
        rpm_left = max(min(speed_rpm - delta, self.rpm_max), 0)
        rpm_right = max(min(speed_rpm + delta, self.rpm_max), 0)
        return rpm_left, rpm_right

    def run(self, duration, cap=0.0, speed_rpm=3000):
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            rpm_left_bar, rpm_right_bar = self.regulation_cap_and_speed(cap, speed_rpm)
            self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            while (time.time() - t0loop) < self.dt:
                time.sleep(0.001)

        self.ard.send_arduino_cmd_motor(0, 0)


if __name__ == '__main__':
    try:
        duration = int(sys.argv[1])
    except:
        duration = math.inf

    ctr = Control()
    ctr.run(duration)
