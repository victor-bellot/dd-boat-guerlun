#
# requires  : gpxpy and pyproj
# sudo apt install python3-pyproj
# sudo apt install python3-gpxpy

import gpxpy
from pyproj import Proj, transform
import numpy as np

#projMeter = Proj("+init=EPSG:2154",preserve_units=False) # Lambert-93 - France 
#projMeter = Proj("+init=EPSG:5627",preserve_units=False) # ED 50
projMeter = Proj("+init=EPSG:32620",preserve_units=False) # UTM Zone 30
projDegre = Proj("+init=EPSG:4326") # WGS84 in degrees and not EPSG:3857 in meters)

# reference point "the flag"
reference_lat = 48.4188
reference_lon = -4.4725
reference_x,reference_y = transform(projDegre,projMeter,reference_lon,reference_lat)
print (reference_x,reference_y,reference_lat,reference_lon)
x = reference_x
y = reference_y
# reference point center of the rugby field
reference_lat = 48.4185
reference_lon = -4.4739
reference_x,reference_y = transform(projDegre,projMeter,reference_lon,reference_lat)
print (reference_x,reference_y,reference_lat,reference_lon)
dx = x - reference_x
dy = y - reference_y
distance = np.sqrt(dx*dx+dy*dy)
print (dx,dy,distance)
exit()

gpx_file = open('tst_ensta_stadium_20210929.gpx', 'r')


init = True
gpx = gpxpy.parse(gpx_file)
for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            #print('Point at ({0},{1}) -> {2}'.format(point.latitude, point.longitude, point.elevation))
            x,y = transform(projDegre,projMeter,point.longitude, point.latitude)
            lat,lon = transform(projMeter,projDegre,x,y)
            print (x,y,lat,lon,point.latitude,point.longitude)
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

