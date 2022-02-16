import time
import sys
import imu9_driver_v2 as imudrv
import signal
import numpy as np

    
def signal_handler(sig, frame):
    print('You pressed Ctrl+C! compute min/max magx/y')
    magxmin = np.min(vx)
    magxmax = np.max(vx)
    magymin = np.min(vy)
    magymax = np.max(vy)
    print (magxmin,magxmax,magymin,magymax)
    sys.exit(0)

imu = imudrv.Imu9IO()
signal.signal(signal.SIGINT, signal_handler)


magx_min=-3468
magx_max=1467
magy_min=-2598
magy_max=1487
imu.fast_heading_calibration (magx_min, magx_max, magy_min, magy_max)
while True:
    magx,magy,magz = imu.read_mag_raw()
    head_deg = imu.heading_deg(magx,magy)
    print (magx,magy,head_deg)
    time.sleep(0.1)

    
