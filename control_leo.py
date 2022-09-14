import sys
import time
import math
import numpy as np
from imu9_driver_v3 import Imu9IO
from tc74_driver_v2 import TempTC74IO
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO
import gps_driver_v2 as gpsdrv
import gpxpy.gpx

data_keys = ['time', 'd_phi', 'rpm_l', 'rpm_r', 'rpm_lb', 'rpm_rb', 'th_l', 'th_r']


def data_to_str(data):
    str_inf = ""
    for k, v in zip(data_keys, data):
        str_inf += k + ': ' + str(int(v)) + ' ; '
    return str_inf[:-3] + '\n'


def cap_to_phi(cap):
    if cap == 'S':
        return np.pi
    elif cap == 'W':
        return np.pi / 2
    elif cap == 'E':
        return -np.pi / 2
    else:
        return 0.0


def delta_odo(odo1, odo0):
    dodo = odo1 - odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


def sawtooth(x):
    return (x + np.pi) % (2 * np.pi) - np.pi


def cvt_gll_ddmm_2_dd(st):
    ilat = st[0]
    ilon = st[2]
    olat = float(int(ilat / 100))
    olon = float(int(ilon / 100))
    olat_mm = (ilat % 100) / 60
    olon_mm = (ilon % 100) / 60
    olat += olat_mm
    olon += olon_mm
    if st[3] == "W":
        olon = -olon
    return olat, olon


class Control:
    def __init__(self, dt=0.5):
        self.ard = ArduinoIO()
        self.enc = EncoderIO()
        self.imu = Imu9IO()
        self.tpr = TempTC74IO()
        self.gps = gpsdrv.GpsIO()

        self.dt = dt
        # set delay between old and new measures : HERE=dt
        self.enc.set_older_value_delay_v2(int(dt * 10))
        self.tpr.set_config(0x0, 0x0)
        self.tpr.set_mode(standby=True, side="both")

        self.cst = {'left': {'kp': 0.01, 'ki': 0.01},
                    'right': {'kp': 0.01, 'ki': 0.01},
                    'phi': {'kp': (3 / 4) / np.pi, 'ki': (3 / 20) / np.pi}}

        self.step_max = 50
        self.u_max = 100
        self.rpm_max = 4000

        self.ei_left, self.ei_right = 0, 0
        self.ei_phi = 0
        self.cmd_left, self.cmd_right = 50, 50

    def reset(self, cmd_left_init=50, cmd_right_init=50):
        self.ei_left, self.ei_right, self.ei_phi = 0, 0, 0
        self.cmd_left, self.cmd_right = cmd_left_init, cmd_right_init

    def change_timing(self, dt):
        self.dt = dt
        self.enc.set_older_value_delay_v2(int(dt * 10))

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

        self.cmd_left = max(min(self.u_max, self.cmd_left + step_left), 0)
        self.cmd_right = max(min(self.u_max, self.cmd_right + step_right), 0)

        self.ard.send_arduino_cmd_motor(self.cmd_left, self.cmd_right)

        print('MEASURED RPM:', rpm_left, rpm_right)
        return rpm_left, rpm_right

    def leo_cap_and_speed(self, delta_phi, rpm_max):
        self.ei_phi += delta_phi * self.dt
        e_phi = self.cst['phi']['kp'] * delta_phi + self.cst['phi']['ki'] * self.ei_phi

        if e_phi >= 0:
            rpm_left_bar = rpm_max - e_phi * rpm_max
            rpm_right_bar = rpm_max
        else:
            rpm_right_bar = rpm_max + e_phi * rpm_max
            rpm_left_bar = rpm_max

        print('RPM BAR:', rpm_left_bar, rpm_right_bar)
        return rpm_left_bar, rpm_right_bar

    def run(self, duration, cap='N', speed_rpm=3000, mission_name='log'):
        # initialisation du fichier .gpx
        gpx = gpxpy.gpx.GPX()
        gpx_track = gpxpy.gpx.GPXTrack()
        gpx.tracks.append(gpx_track)
        gpx_segment = gpxpy.gpx.GPXTrackSegment()
        gpx_track.segments.append(gpx_segment)
        """
        file = open(log + '.txt', 'a')
        file.write('duration: ' + str(duration) + ' ; ' + 'cap: ' + cap + ' ; spd: ' + str(speed_rpm) + '\n')
        """
        self.reset()
        t0 = time.time()
        while (time.time() - t0) < duration:
            t0loop = time.time()

            phi = self.get_current_cap()
            delta_phi = sawtooth(cap_to_phi(cap) - phi)

            rpm_left_bar, rpm_right_bar = self.leo_cap_and_speed(delta_phi, speed_rpm)
            #rpm_left, rpm_right = self.regulation_rpm(rpm_left_bar, rpm_right_bar)

            #on enregistre les points gps
            gps_data_string = self.gps.read_gll()
            if not (gps_data_string[0] == 0.0):
                lat, lon = cvt_gll_ddmm_2_dd(gps_data_string)
                gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon))
            """
            temp_left, temp_right = self.tpr.read_temp()
            data = [(t0loop - t0) * 1000, delta_phi * (180 / np.pi), rpm_left, rpm_right,
                    rpm_left_bar, rpm_right_bar, temp_left, temp_right]
            information = data_to_str(data)
            file.write(information)
            """
            while (time.time() - t0loop) < self.dt:
                time.sleep(0.001)

        self.ard.send_arduino_cmd_motor(0, 0)
        #file.close()

        fp = open("tst_leo.gpx", "w")
        fp.write(gpx.to_xml())
        fp.write("\n")
        fp.close()


if __name__ == '__main__':
    # Chose a duration
    try:
        d = int(sys.argv[1])
    except:
        d = math.inf

    # Chose a cap
    try:
        c = str(sys.argv[2])
    except:
        c = 'N'

    # Chose a speed
    try:
        s = int(sys.argv[3])
    except:
        s = 0

    ctr = Control()

    if d < 0:
        d = abs(d)
        ctr.run(d, speed_rpm=s, cap='N')
        ctr.run(d, speed_rpm=s, cap='W')
        ctr.run(d, speed_rpm=s, cap='S')
        ctr.run(d, speed_rpm=s, cap='E')
    else:
        ctr.run(d, speed_rpm=s, cap=c)
