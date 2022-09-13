import sys
import time
import numpy as np
from imu9_driver_v3 import Imu9IO


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
        n = 200
        measurements = np.zeros((3, n))
        for k in range(n):
            measurements[:, k] = np.array(imu.read_mag_raw())
            time.sleep(0.01)
        print(np.median(measurements, axis=1))
