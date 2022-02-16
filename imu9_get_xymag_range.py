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

vx=[]
vy=[]
while True:
    magx,magy,magz = imu.read_mag_raw()
    vx.append(magx)
    vy.append(magy)
    magxmin = np.min(vx)
    magxmax = np.max(vx)
    magymin = np.min(vy)
    magymax = np.max(vy)
    print (magx,magy,magz,magxmin,magxmax,magymin,magymax)
    time.sleep(0.1)

    
