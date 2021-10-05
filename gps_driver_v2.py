import serial
import os
import time

# the GPS sensor gives informations using the NMEA standard
# https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard
# https://en.wikipedia.org/wiki/NMEA_0183

class GpsIO:
    def __init__(self):
        # open serial line connected to the GPS sensor
        self.init_line()
        #time.sleep(1.0)
        #print(ser)
   
    def init_line(self,timeout=1.0):
        self.ser = serial.Serial('/dev/ttyS0',timeout=timeout)

    def init_line_devname_baudrate(self,devname,baudrate,timeout=1.0):
        self.ser = serial.Serial(devname,baudrate=baudrate,timeout=timeout,xonxoff=False, rtscts=False, dsrdtr=False)
        
    def close(self):
        self.ser.close()

        
    def read_next_message(self):
        v=self.ser.readline().decode("utf-8")
        #print (v)
        return v

    # read the position in the GPGLL message
    # by default one GPGLL message is expected every 20 messages
    # warning: blocking function, not to use in control loops 
    def read_gll(self,n_try_max=20):
        val=[0.,'N',0.,'W',0.]
        for i in range(n_try_max):
            rdok = True
            try:
                v=self.ser.readline().decode("utf-8")
            except:
                print ("error reading GPS !!")
                rdok = False
                break # go out
            if rdok:
                if str(v[0:6]) == "$GPGLL":
                    vv = v.split(",")
                    if len(vv[1]) > 0:
                        val[0] = float(vv[1])
                    if len(vv[2]) > 0:
                        val[1] = vv[2]
                    if len(vv[3]) > 0:
                        val[2] = float(vv[3])
                    if len(vv[4]) > 0:
                        val[3] = vv[4]
                    if len(vv[5]) > 0:
                        val[4] = float(vv[5])
                    break # GPGLL found !  exit !
        return val

    def read_gll_non_blocking(self,timeout=0.01):
        self.ser.timeout=timeout
        v=self.ser.readline()
        msg=False
        val=[0.,'N',0.,'W',0.]
        if len(v)>0:
            st=v.decode("utf-8")
            if str(st[0:6]) == "$GPGLL":
                vv = st.split(",")
                if len(vv[1]) > 0:
                    val[0] = float(vv[1])
                if len(vv[2]) > 0:
                    val[1] = vv[2]
                if len(vv[3]) > 0:
                    val[2] = float(vv[3])
                if len(vv[4]) > 0:
                    val[3] = vv[4]
                if len(vv[5]) > 0:
                    val[4] = float(vv[5])
                msg=True
        return msg,val
        
    
if __name__ == "__main__":
    gps = GpsIO()

    # display the 20 first messages
    #for i in range(20):
    #    print (gps.read_next_message())

    # display the 20 positions (GPGLL) messages
    #for i in range(20):
    #    print (gps.read_gll())

    # test non blocking read for 20 positions
    cnt=0
    while True:
        gll_ok,gll_data=gps.read_gll_non_blocking()
        if gll_ok:
            print (gll_data)
            cnt += 1
            if cnt==20:
                break
        time.sleep(0.01)

    gps.close()
