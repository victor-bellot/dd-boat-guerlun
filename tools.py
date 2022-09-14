import gpxpy.gpx
import numpy as np
from gps_driver_v2 import GpsIO

data_keys = ['time', 'd_phi', 'rpm_l', 'rpm_r', 'rpm_lb', 'rpm_rb', 'th_l', 'th_r']

coordinates = {'ponton': (48.199024, -3.014790),
               'nord': (48.199817, -3.015603),
               'ouest': (48.199038, -3.015807),
               'plage': (48.199807, -3.014803)}

rho = 154700  # to check


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


def convert(data):
    lx_raw = data[0]
    ly_raw = data[2]
    lx = lx_raw // 100
    ly = ly_raw // 100
    lx += (lx_raw % 100) / 60
    ly += (ly_raw % 100) / 60
    if data[3] == 'W':
        ly = -ly
    return lx, ly


# coord : spherical ; pos : cartesian
def coord_to_pos(coords, origin='ponton'):
    lx, ly = coords
    lxo, lyo = coordinates[origin]
    x = rho * np.cos(ly) * (lx - lxo)
    y = rho * (ly - lyo)
    return np.array([[x], [y]])


class GpsManager:
    def __init__(self):
        self.gps = GpsIO()
        self.gpx = gpxpy.gpx.GPX()
        self.gpx_track = gpxpy.gpx.GPXTrack()
        self.gpx.tracks.append(self.gpx_track)
        self.gpx_segment = gpxpy.gpx.GPXTrackSegment()
        self.gpx_track.segments.append(self.gpx_segment)

    def draw_point(self):
        gps_data_string = self.gps.read_gll()
        if not (gps_data_string[0] == 0.0):
            lat, lon = cvt_gll_ddmm_2_dd(gps_data_string)
            self.gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon))

    def get_coord(self):
        msg, data = self.gps.read_gll_non_blocking()
        if msg:
            return convert(data)


class Line:
    def __init__(self, name0, name1, coord0=None, coord1=None):
        self.name0 = name0
        self.name1 = name1

        if name0 in coordinates.keys():
            self.coord0 = coordinates[name0]
        else:
            self.coord0 = coord0

        if name1 in coordinates.keys():
            self.coord1 = coordinates[name1]
        else:
            self.coord1 = coord1

        self.pos0 = coord_to_pos(self.coord0)
        self.pos1 = coord_to_pos(self.coord1)

        delta_pos = self.pos1 - self.pos0
        self.direction = delta_pos / np.linalg.norm(delta_pos)

        theta = np.arctan2(self.direction[1], self.direction[0])
        self.normal = np.array([-np.sin(theta), np.cos(theta)])
