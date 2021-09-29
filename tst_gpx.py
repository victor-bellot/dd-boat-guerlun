import time
import sys
import gps_driver_v2 as gpsdrv

def cvt_gll_ddmm_2_dd (st):
    sts =st.split(" ")
    stlat = sts[0]
    stlon = sts[2]
    stlats = stlat.split(".")
    stlons = stlon.split(".")
    lat = double(stlats[0][0:len(stlats[0])-2])
    lon = double(stlons[0][0:len(stlons[0])-2])
    print (lat,lon)

gps = gpsdrv.GpsIO()
# debug with USB GPS
gps.init_line_devname_baudrate("/dev/ttyUSB0",9600)

tmax = 25.0
t0 = time.time()
while True:
    print ("---------------------------------------------------")

    # test GPS
    gps_data_string = gps.read_gll()
    print ("GPS:",gps_data_string)

    
