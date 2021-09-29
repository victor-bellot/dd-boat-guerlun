#
# requires  : gpxpy and pyproj
# sudo apt install python3-pyproj
# sudo apt install python3-gpxpy

import gpxpy
import gpxpy.gpx
from pyproj import Proj, transform
import numpy as np

projMeter = Proj("+init=EPSG:2263",preserve_units=True)
projDegre = Proj("+init=EPSG:4326") # WGS84 in degrees and not EPSG:3857 in meters)

gpx = gpxpy.gpx.GPX()
# Create first track in our GPX:
gpx_track = gpxpy.gpx.GPXTrack()
gpx.tracks.append(gpx_track)
# Create first segment in our GPX track:
gpx_segment = gpxpy.gpx.GPXTrackSegment()
gpx_track.segments.append(gpx_segment)

# reference point "the flag"
reference_lat = 48.4188
reference_lon = -4.4725
gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(reference_lat, reference_lon))
reference_lat = 48.4185
reference_lon = -4.4739
gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(reference_lat, reference_lon))
reference_x,reference_y = transform(projDegre,projMeter,reference_lat,reference_lon)

fp = open("reference.gpx","w")
fp.write(gpx.to_xml())
fp.write("\n")
fp.close()


gpx_file = open('tst_ensta_stadium_20210929.gpx', 'r')

gpx = gpxpy.parse(gpx_file)
for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            #print('Point at ({0},{1}) -> {2}'.format(point.latitude, point.longitude, point.elevation))
            x,y = transform(projDegre,projMeter,point.latitude, point.longitude)
            lat,lon = transform(projMeter,projDegre,x,y)
            #print (x,y,lat,lon)
            dx = x - reference_x
            dy = y - reference_y
            distance = np.sqrt(dx*dx+dy*dy)
            print (dx,dy,distance)



# test project lat,lon (deg) to meters
#from pyproj import Proj, transform
#inProj  = Proj("+init=EPSG:2263",preserve_units=True)
#outProj = Proj("+init=EPSG:4326") # WGS84 in degrees and not EPSG:3857 in meters)
#x2,y2 = -74.2700000001129, 40.46999999990434
#print (x2,y2)
#x1,y1 = transform(outProj,inProj,x2,y2)
#print (x1,y1)
#print(transform(inProj,outProj,x1,y1))

