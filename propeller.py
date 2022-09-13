import sys
import time
from arduino_driver_v2 import ArduinoIO
from encoders_driver_v2 import EncoderIO

if __name__ == "__main__":
    try:
        cmdl = int(sys.argv[1])
    except:
        cmdl = 30
    try:
        cmdr = int(sys.argv[2])
    except:
        cmdr = 30

    dt = 0.2
    arduino = ArduinoIO()
    encoder = EncoderIO()

    arduino.send_arduino_cmd_motor(cmdl, cmdr)

    for _ in range(10):
        delta_odo_left, delta_odo_right = encoder.get_odo_delta(dt)
        print("Delta odo left:", delta_odo_left)
        print("Delta odo right:", delta_odo_right)

        print("---")
        time.sleep(0.1)

    arduino.send_arduino_cmd_motor(0, 0)
    sys.exit(0)
