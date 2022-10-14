from drivers.arduino_driver_v2 import ArduinoIO

if __name__ == "__main__":
    """
    Stop the robot in case of emergency
    """
    ard = ArduinoIO()
    ard.send_arduino_cmd_motor(0, 0)
