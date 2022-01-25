import struct
import time
import sys
import math
import i2creal as i2c  # currently only real I2C on ddboats (no simulated I2C)

# LIS3DML 0x1e  (mag sensor)
# LSM6    0x6b  (accelero - gyro)

class TempLC74IO():
    def __init__(self):
        self.__bus_nb = 1  # 1 on DDBoat, 2 on DartV2
        self.__addr = 0x4b  # temp LC 74 sensor
        self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
        self.temp = 0

        # configure (to be done !)
        # self.__dev_i2c.write(0x01,[0x00])

    def read_temp(self):
        self.temp = self.__dev_i2c.read(0x00,2)
        return self.temp
 
if __name__ == "__main__":
    temperature = TempLC74IO()
    for i in range(20):
        print (temperature.read_temp())
        time.sleep(0.25)

