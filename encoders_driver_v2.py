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
    def __init__(self,dev_tty=0):
        self.baud_rate = 115200
        self.voltLeftFilt = 500.0
        self.voltRightFilt = 500.0
        self.a = 0.99
        self.sync = False
        # raw device : direct access to the device
        # warning : data lags if not read fast enough !!! 
        # to be compatible with old codes
        self.dev_tty = '/dev/ttyENCRAW'
        # advanced device : replicated on 3 ports /dev/ttyENC[0-2] 
        # no more lags !
        # send a command on the terminal
        #   "C"  : return the last measured values (vector of 5 values) 
        #             time,dir_left,dir_right,odo_left,odo_right
        #   "P"  : return the last and the nth old values
        #             time0,dir_left0,dir_right0,odo_left0,odo_right0
        #             time1,dir_left1,dir_right1,odo_left1,odo_right1
        #          this allows to compute the time and odo differences
        #              delta_time = time0 - time1
        #              delta_odo_left = odo_left0 - odo_left1
        #              delta_odo_right = odo_right0 - odo_right1
        #   "Dn;" or "Dnn;" : set the nth old value 1<n<n or 01<<nn<99
        #          by default nn=20
        self.dev_tty_v2 = '/dev/ttyENC0'
        if dev_tty == 1:
            self.dev_tty_v2 = '/dev/ttyENC1'
        if dev_tty == 2:
            self.dev_tty_v2 = '/dev/ttyENC2'
        self.init_line()

    def init_line(self,timeout=1.0):
        self.ser = serial.Serial(self.dev_tty,self.baud_rate,timeout=timeout)

    # in principle , no need to use this function , baudrate is normally 115200 with
    # the current software version
    def set_baudrate(self,baudrate=115200):
        self.baud_rate = baudrate
        st = os.system ("stty -F %s %d"%(self.dev_tty,self.baud_rate))
        print (st)
        st = os.system ("stty -F %s"%(self.dev_tty))
        print (st)

    def init_line_v2(self,timeout=1.0):
        self.ser = serial.Serial(self.dev_tty_v2,self.baud_rate,timeout=timeout)

    def set_baudrate_v2(self,baudrate=115200):
        self.baud_rate = baudrate
        st = os.system ("stty -F %s %d"%(self.dev_tty_v2,self.baud_rate))
        print (st)
        st = os.system ("stty -F %s"%(self.dev_tty_v2))
        print (st)

    def close_line(self):
        self.ser.close()
    
    # find the sync chars 0xFF and 0x0d at the beginning of the data
    def get_sync(self, max_tryout=25):
        tryout = 0
        while True:
            c1 = self.ser.read(1)
            if len(c1) == 0: # Catch the case c1=b''
                tryout += 1
                if tryout > max_tryout:
                    self.sync = False
                    print("sync error : [get_sync() reach max tryout] please check line")
                    break
            else:
                # Core of get_sync()
                if ord(c1) == 0xff:
                    c2 = self.ser.read(1)
                    if ord(c2) == 0x0d:
                        v = self.ser.read(15)
                        self.sync = True
                        break
            time.sleep(0.001)

    # read next packet, assume sync has been done before
    # detect if sync is lost (sync false at return)
    def read_packet(self,debug=False):
        # check sync
        if not self.sync:
            self.get_sync()
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
          self.sync = False
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
          self.voltLeftFilt = self.voltLeftFilt * self.a + voltLeft * (1.0 - self.a) 
          self.voltRightFilt = self.voltRightFilt * self.a + voltRight * (1.0 - self.a)
          if debug:
              print (timer,sensLeft,sensRight,posLeft,posRight,
                     voltLeft,voltRight,'[',
                     int(round(self.voltLeftFilt)),
                     int(round(self.voltRightFilt)),']',stc3)
        return self.sync,data

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

    # get last value on V2 device
    def get_last_value_v2 (self):
        v=self.ser.write(b'C')
        st=[]
        st.append(self.ser.read(1))
        while True:
            ch = self.ser.read(1)
            if cf == '\0':
                break
            else:
                st.append(ch)
        return st



if __name__ == "__main__":
    encoddrv = EncoderIO()
        
    # test raw encoder data (old version, mind the potentiel time lag if read is not fast enough !)
    cnt = 0
    encoddrv.get_sync()
    while cnt<10:
        sync,data_encoders = encoddrv.read_packet(debug=True)
        print (sync,data_encoders)
        if not sync:
            break
        cnt += 1

    # test the new version
    cnt = 0
    while cnt<1:
        data_encoders = encoddrv.get_last_value_v2()
        print ()



