
import struct
import time
import sys
import math
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

# Microchip TC74 temperature sensor

class TempTC74IO():
    def __init__(self):
        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr_left = 0x4d  # temp LC 74 sensor left
        self.__addr_right = 0x48  # temp LC 74 sensor right
        self.__dev_i2c_left=i2c.i2c(self.__addr_left,self.__bus_nb)
        self.__dev_i2c_right=i2c.i2c(self.__addr_right,self.__bus_nb)
        self.__temp_right = 0
        self.__cfg_right = 0
        self.__temp_left = 0
        self.__cfg_left = 0
        self.set_mode(standby=False)

    def set_mode (self,standby=False,side="both"):
        if side == "left" or side =="both":
            if standby:
                self.__dev_i2c_left.write(0x01,[0x80])
            else:
                self.__dev_i2c_left.write(0x01,[0x00])
        if side == "right" or side =="both":
            if standby:
                self.__dev_i2c_right.write(0x01,[0x80])
            else:
                self.__dev_i2c_right.write(0x01,[0x00])
        self.__cfg_left, self.__cfg_right = self.get_config()

    def set_config(self,cfg_left, cfg_right):
        self.__cfg_left = cfg_left
        self.__dev_i2c_left.write(0x01,[cfg])
        self.__cfg_right = cfg_right
        self.__dev_i2c_right.write(0x01,[cfg])

    def get_config(self):
        self.__cfg_left = self.__dev_i2c_left.read(0x01,1)[0]
        self.__cfg_right = self.__dev_i2c_right.read(0x01,1)[0]
        return self.__cfg_left, self.__cfg_right

    def read_temp(self):
        self.__temp_left = self.__dev_i2c_left.read(0x00,1)[0]
        self.__temp_right = self.__dev_i2c_right.read(0x00,1)[0]
        return self.__temp_left, self.__temp_right 
 
if __name__ == "__main__":
    temperature = TempTC74IO()

    cfg_left, cfg_right = temperature.get_config()
    print ("Config: 0x%2.2x 0x%2.2x"%(cfg_left, cfg_right))

    time.sleep(1.0)

    temperature.set_mode(standby=True,side="both")
    cfg_left, cfg_right = temperature.get_config()
    print ("Config: 0x%2.2x 0x%2.2x"%(cfg_left, cfg_right)) 

    for i in range(5):
        temp_left, temp_right = temperature.read_temp()
        print ("Temperature (stand by) : Left=%d deg.C, Right=%d deg.C"%(temp_left, temp_right))
        time.sleep(1.0)

    temperature.set_mode(standby=False,side="both")
    cfg_left, cfg_right = temperature.get_config()
    print ("Config: 0x%2.2x 0x%2.2x"%(cfg_left, cfg_right)) 
    for i in range(55):
        temp_left, temp_right = temperature.read_temp()
        print ("Temperature : Left=%d deg.C, Right=%d deg.C"%(temp_left, temp_right))
        time.sleep(1.0)

    temperature.set_mode(standby=True,side="both")
    cfg_left, cfg_right = temperature.get_config()
    print ("Config: 0x%2.2x 0x%2.2x"%(cfg_left, cfg_right)) 

