import math
import sys
import numpy as np
import time
import arduino_driver_v2
import encoders_driver_v2


def delta_odo(odo1, odo0):
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def get_rpm(enc, dt):
    # 1 : new ; 0 : old
    st1, st0 = enc.get_last_and_older_values_v2()
    data_encoders0 = np.array(st0.split(",")).astype(np.float)
    data_encoders1 = np.array(st1.split(",")).astype(np.float)

    odo_left0 = data_encoders0[4]
    odo_right0 = data_encoders0[3]

    odo_left1 = data_encoders1[4]
    odo_right1 = data_encoders1[3]

    rpm_left = delta_odo(odo_left1, odo_left0) / 8.0 / dt * 60.0
    rpm_right = delta_odo(odo_right1, odo_right0) / 8.0 / dt * 60.0

    return rpm_left, rpm_right


def regul(rpm_left_bar, rpm_right_bar, dt=1.0, cmd_left_init=50, cmd_right_init=50):
    ard = arduino_driver_v2.ArduinoIO()
    enc = encoders_driver_v2.EncoderIO()

    kp_left, ki_left = 1, 0.01
    kp_right, ki_right = 1, 0.01

    ei_left, ei_right = 0, 0
    cmd_left, cmd_right = cmd_left_init, cmd_right_init

    ard.send_arduino_cmd_motor(cmd_left, cmd_right)
    enc.set_older_value_delay_v2(10)  # number of measurement during dt

    t0 = time.time()
    while (time.time() - t0) < duration:
        t0loop = time.time()
        rpm_left, rpm_right = get_rpm(enc, dt)

        # left motor
        e_left = rpm_left_bar - (-rpm_left)
        ei_left += e_left * dt
        u_left = kp_left * e_left + ki_left * ei_left

        # right motor
        e_right = rpm_right_bar - rpm_right
        ei_right += e_right * dt
        u_right = kp_right * e_right + ki_right * ei_right

        print(rpm_left, rpm_right, u_left, u_right)

        # On seuil la variation en tension
        if abs(u_left) > 60:
            u_left = 60 * u_left/abs(u_left)
        if abs(u_right) > 60:
            u_right = 60 * u_right/abs(u_right)

        cmd_left += u_left
        cmd_right += u_right

        ard.send_arduino_cmd_motor(cmd_left, cmd_right)

        while (time.time() - t0loop) < dt:
            time.sleep(0.001)

    ard.send_arduino_cmd_motor(0, 0)


if __name__ == '__main__':
    try:
        duration = int(sys.argv[1])
    except:
        duration = math.inf

    regul(rpm_left_bar=10000, rpm_right_bar=10000)
