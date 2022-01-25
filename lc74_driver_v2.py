import struct
import time
import sys
import math
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

class TempLC74IO():
    def __init__(self):
        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr = 0x4d  # temp LC 74 sensor
        self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
        self.__temp = 0
        self.__cfg = 0
        self.set_mode(standby=False)

    def set_mode (self,standby=False):
        if standby:
            self.__dev_i2c.write(0x01,[0x80])
            self.__cfg = self.get_config()
        else:
            self.__dev_i2c.write(0x01,[0x00])
            self.__cfg = self.get_config()

    def set_config(self,cfg):
        self.__cfg = cfg
        #self.__dev_i2c.write(0x01,[self.__cfg])
        self.__dev_i2c.write(0x01,[cfg])

    def get_config(self):
        self.__cfg = self.__dev_i2c.read(0x01,1)[0]
        return self.__cfg

    def read_temp(self):
        self.__temp = self.__dev_i2c.read(0x00,1)[0]
        return self.__temp 
 
if __name__ == "__main__":
    temperature = TempLC74IO()
    print ("Config: 0x%2.2x"%(temperature.get_config()))
    temperature.set_mode(standby=True)
    print ("Config: 0x%2.2x"%(temperature.get_config()))
    for i in range(5):
        print ("T = %d deg.C"%(temperature.read_temp()))
        time.sleep(1.0)
    temperature.set_mode(standby=False)
    print ("Config: 0x%2.2x"%(temperature.get_config()))
    for i in range(25):
        print ("T = %d deg.C"%(temperature.read_temp()))
        time.sleep(1.0)
    temperature.set_mode(standby=True)
    print ("Config: 0x%2.2x"%(temperature.get_config()))


