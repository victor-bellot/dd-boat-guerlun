import time
import numpy as np
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO


def delta_odo(odo1, odo0):
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


class Control:
    def __init__(self, dt=0.5, measurement_per_dt=10):
        self.ard = ArduinoIO()
        self.enc = EncoderIO()

        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * measurement_per_dt))

        self.cst = {'left': {'kp': 0.01, 'ki': 0.01},
                    'right': {'kp': 0.01, 'ki': 0.01}}

        self.ei_left, self.ei_right = 0, 0
        self.cmd_left, self.cmd_right = 50, 50

    def reset(self, cmd_left_init=50, cmd_right_init=50):
        self.ei_left, self.ei_right = 0, 0
        self.cmd_left, self.cmd_right = cmd_left_init, cmd_right_init

    def change_timing(self, dt, measurement_per_dt):
        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * measurement_per_dt))

    def get_rpm(self):
        # 1 : new ; 0 : old
        st1, st0 = self.enc.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(",")).astype(np.float)
        data_encoders1 = np.array(st1.split(",")).astype(np.float)

        odo_left0 = data_encoders0[4]
        odo_right0 = data_encoders0[3]

        odo_left1 = data_encoders1[4]
        odo_right1 = data_encoders1[3]

        rpm_left = (8. / 60.) * delta_odo(odo_left1, odo_left0) / self.dt
        rpm_right = (8. / 60.) * delta_odo(odo_right1, odo_right0) / self.dt

        return rpm_left, rpm_right

    def regulation_rpm(self, rpm_left_bar, rpm_right_bar):
        rpm_left, rpm_right = self.get_rpm()

        # left motor
        e_left = rpm_left_bar - rpm_left
        self.ei_left += e_left * self.dt
        u_left = self.cst['left']['kp'] * e_left + self.cst['left']['ki'] * self.ei_left

        # right motor
        e_right = rpm_right_bar - (-rpm_right)
        self.ei_right += e_right * self.dt
        u_right = self.cst['right']['kp'] * e_right + self.cst['right']['ki'] * self.ei_right

        # On seuil la variation en tension
        if abs(u_left) > 60:
            u_left = 60 * u_left / abs(u_left)
        if abs(u_right) > 60:
            u_right = 60 * u_right / abs(u_right)

        self.cmd_left = min(150, self.cmd_left + u_left)
        self.cmd_right = min(150, self.cmd_right + u_right)

        print(rpm_left, rpm_right, u_left, u_right, self.cmd_left, self.cmd_right)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

    def regulation_cap(self, cap):
        return 3000, 3000

    def run(self, duration, cap=0.0):
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            rpm_left_bar, rpm_right_bar = self.regulation_cap(cap)
            self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            while (time.time() - t0loop) < self.dt:
                time.sleep(0.001)

        self.ard.send_arduino_cmd_motor(0, 0)


if __name__ == '__main__':
    ctr = Control()
    ctr.run(10)
