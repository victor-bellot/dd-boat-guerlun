#
# requires  : sudo apt install python3-pyproj 

# test project lat,lon (deg) to meters
from pyproj import Proj, transform
inProj  = Proj("+init=EPSG:2263",preserve_units=True)
outProj = Proj("+init=EPSG:4326") # WGS84 in degrees and not EPSG:3857 in meters)
x2,y2 = -74.2700000001129, 40.46999999990434
print (x2,y2)
x1,y1 = transform(outProj,inProj,x2,y2)
print (x1,y1)
print(transform(inProj,outProj,x1,y1))

