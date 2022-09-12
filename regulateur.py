import numpy as np
import time
import arduino_driver_v2
import encoders_driver_v2


def regul(w1_bar, w2_bar):
    kp1, ki1 = 1, 0.01
    kp2, ki2 = 1, 0.01
    ei1, ei2 = 0, 0
    i = 0
    while i < 50:
        delta_left, delta_right = enc.get_odo_delta(dt)

        # moteur 1 left
        w1 = 2**13/dt * np.sin(delta_left / 2**16)
        e1 = w1_bar - (-w1)
        ei1 += e1 * dt
        u1 = kp1 * e1 + ki1 * ei1

        # moteur 2 right
        w2 = 2 ** 13 / dt * np.sin(delta_right / 2 ** 16)
        e2 = w2_bar - w2
        ei2 += e2 * dt
        u2 = kp2 * e2 + ki2 * ei2

        print(w1, w2, u1, u2, delta_left, delta_right)

        if abs(u1) > 60:
            u1 = 60 * u1/abs(u1)
        if abs(u2) > 60:
            u2 = 60 * u2/abs(u2)

        ard.send_arduino_cmd_motor(0, u2)
        i += 1


if __name__ == '__main__':
    ard = arduino_driver_v2.ArduinoIO()
    enc = encoders_driver_v2.EncoderIO()

    dt = 0.1
    cap = 0
    w1_bar, w2_bar = 0, 30

    regul(w1_bar, w2_bar)
    ard.send_arduino_cmd_motor(0, 0)



