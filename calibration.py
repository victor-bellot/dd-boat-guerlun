import sys
import time
import numpy as np
from imu9_driver_v2 import Imu9IO


if __name__ == "__main__":
    try:
        correction = int(sys.argv[1])
    except:
        correction = None

    imu = Imu9IO()

    if correction:
        while True:
            print(imu.correction_mag().flatten(), imu.orientation() * (180 / np.pi))
            time.sleep(0.1)
    else:
        acc = np.zeros((3, 1))
        for i in range(200):
            acc += np.array(imu.read_mag_raw()).reshape(-1, 1)
            time.sleep(0.01)
        print(acc / 200)
