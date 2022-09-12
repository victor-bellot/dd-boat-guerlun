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


def get_rpm():
    st1, st0 = enc.get_last_and_older_values_v2()
    data_encoders0 = np.array(st0.split(",")).astype(np.float)
    data_encoders1 = np.array(st1.split(",")).astype(np.float)

    posLeft0 = data_encoders0[4]
    posRight0 = data_encoders0[3]

    posLeft1 = data_encoders1[4]
    posRight1 = data_encoders1[3]

    rpm_left = delta_odo(posLeft1, posLeft0) / 8.0 / T_loop * 60.0
    rpm_right = delta_odo(posRight1, posRight0) / 8.0 / T_loop * 60.0

    return rpm_left, rpm_right


def regul():
    kp_left, ki_left = 1, 0.01
    kp_right, ki_right = 1, 0.01

    ei_left, ei_right = 0, 0

    t0 = time.time()
    while (time.time() - t0) < duration:
        t0loop = time.time()
        rpm_left, rpm_right = get_rpm()

        # left motor
        e_left = rpm_left_bar - (-rpm_left)
        ei_left += e_left * dt
        u_left = kp_left * e_left + ki_left * ei_left

        # right motor
        e_right = rpm_right_bar - rpm_right
        ei_right += e_right * dt
        u_right = kp_right * e_right + ki_right * ei_right

        print(rpm_left, rpm_right, u_left, u_right)

        # ABSURDE : on décélère, on ne tourne pas dans l'autre sens!
        if abs(u_left) > 255:
            u_left = 60 * u_left/abs(u_left)
        if abs(u_right) > 60:
            u_right = 60 * u_right/abs(u_right)

        ard.send_arduino_cmd_motor(u_left, u_right)

        while (time.time() - t0loop) < T_loop:
            time.sleep(0.001)


if __name__ == '__main__':
    ard = arduino_driver_v2.ArduinoIO()
    enc = encoders_driver_v2.EncoderIO()

    dt = 0.1
    cap = 0
    cmd_left, cmd_right = 50, 50
    rpm_left_bar, rpm_right_bar = 10000, 10000

    duration = 10.0
    T_loop = 1.0
    ard.send_arduino_cmd_motor(cmd_left, cmd_right)

    enc.set_older_value_delay_v2(10)  # 50 measurements -> 5s (10 meas/s)

    regul()
    ard.send_arduino_cmd_motor(0, 0)
