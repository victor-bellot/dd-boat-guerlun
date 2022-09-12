import numpy as np
import time
import arduino_driver_v2
import encoders_driver_v2


def regul(w_left_bar, w_right_bar):
    kp_left, ki_left = 1, 0.01
    kp_right, ki_right = 1, 0.01

    ei_left, ei_right = 0, 0
    i = 0

    while i < 50:
        w_left, w_right = enc.get_rps(dt)

        # left motor
        e_left = w_left_bar - (-w_left)
        ei_left += e_left * dt
        u_left = kp_left * e_left + ki_left * ei_left

        # right motor
        e_right = w_right_bar - w_right
        ei_right += e_right * dt
        u_right = kp_right * e_right + ki_right * ei_right

        print(w_left, w_right, u_left, u_right)

        if abs(u_left) > 60:
            u_left = 60 * u_left/abs(u_left)
        if abs(u_right) > 60:
            u_right = 60 * u_right/abs(u_right)

        ard.send_arduino_cmd_motor(u_left, u_right)

        i += 1


if __name__ == '__main__':
    ard = arduino_driver_v2.ArduinoIO()
    enc = encoders_driver_v2.EncoderIO()

    dt = 0.1
    cap = 0
    w1_bar, w2_bar = 200, 200

    # ard.send_arduino_cmd_motor(50, 50)
    # for _ in range(100):
    #     delta_left, delta_right = enc.get_odo_delta(dt)
    #     rpm_left = (60 / 8) * (delta_left / dt)
    #     rpm_right = (60 / 8) * (delta_right / dt)
    #     print(rpm_left, rpm_right)

    regul(w1_bar, w2_bar)
    ard.send_arduino_cmd_motor(0, 0)
