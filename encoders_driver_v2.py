import serial
import os
import time
import struct

# encoder frame
#  0     : sync 0xFF
#  1     : sync 0x0D
#  2-5   : timer MSByte first
#  6     : direction 1 (left)
#  7     : direction 2 (right)
#  8-9   : encoder 1 (left)
#  10-11 : encoder 2 (right)
#  12-13 : Hall voltage 1 (left)
#  14-15 : Hall voltage 2 (right)
#  16    : sync 0xAA

# data = [timer,dirLeft,dirRight,encLeft,encRight,voltLeft,voltRight]

class EncoderIO():
    def __init__(self):
        self.baud_rate = 115200
        self.init_line()

    def init_line(self,timeout=1.0):
        self.ser = serial.Serial('/dev/ttyUSB0',self.baud_rate,timeout=timeout)

    # in principle , no need to use this function , baudrate is normally 115200 with
    # the current software version
    def set_baudrate(self,baudrate=115200):
        self.baud_rate = baudrate
        st = os.system ("stty -F /dev/ttyUSB0 %d"%(self.baud_rate))
        print (st)
        st = os.system ("stty -F /dev/ttyUSB0")
        print (st)

    def close_line(self):
        self.ser.close()

    # find the sync chars 0xFF and 0x0d at the beginning of the data
    def get_sync(self):
        while True:
            c1 = self.ser.read(1)
            if ord(c1) == 0xff:
                c2 = self.ser.read(1)
                if ord(c2) == 0x0d:
                    v = self.ser.read(15)
                    break

    # read next packet, assume sync has been done before
    # detect if sync is lost (sync false at return)
    def read_packet(self,debug=False):
        sync = True  # assume sync has been done before
        data = []
        v=self.ser.read(17)
        #print (type(v))
        #st=""
        #for i in range(len(v)):
        #  st += "%2.2x"%(ord(v[i]))
        #print st
        c1 = v[0]
        c2 = v[1]
        if (c1 != 0xff) or (c2 != 0x0d):
          if debug:
              print ("sync lost, exit")
          sync = False
        else:
          timer = (v[2] << 32)
          timer += (v[3] << 16)
          timer += (v[4] << 8)
          timer += v[5]
          sensLeft = v[7]
          sensRight= v[6]
          posLeft = v[10] << 8
          posLeft += v[11]
          posRight = v[8] << 8
          posRight += v[9]
          voltLeft = v[14] << 8
          voltLeft += v[15]
          voltRight = v[12] << 8
          voltRight += v[13]
          c3 = v[16]
          stc3 = "%2.2X"%(c3)
          data.append(timer)
          data.append(sensLeft)
          data.append(sensRight)
          data.append(posLeft)
          data.append(posRight)
          data.append(voltLeft)
          data.append(voltRight)
          if debug:
              print (timer,sensLeft,sensRight,posLeft,posRight,voltLeft,voltRight,stc3)
        return sync,data

    # do everything (open, sync, read, close) once (mainly for debug purpose)
    def read_single_packet(self,debug=True):
        self.init_line()
        self.get_sync(ser)
        sync,data = self.read_packet(ser,debug=debug)
        self.close_line(ser)
        timeAcq = data[0]
        sensLeft = data[1]
        sensRight = data[2]
        posLeft = data[3]
        posRight = data[4]
        return sync, timeAcq, sensLeft, sensRight, posLeft, posRight

if __name__ == "__main__":
    encoddrv = EncoderIO()
        
    while True:
        encoddrv.get_sync()
        while True:
            sync,data_encoders = encoddrv.read_packet(debug=True)
            if not sync:
                break

