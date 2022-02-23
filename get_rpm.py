import encoder_driver_v2 as encodrv
import arduino_driver_py3 as ardudrv
import sys
import time

def delta_odo (odo1,odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


if __name__ == "__main__":
    
    timeout = 1.0
    cmdl = 40
    cmdr = 40
    duration = 20.0
    tloop = 1.0 # 1 Hz loop
    encodrv.set_baudrate(baudrate=115200)
    serial_arduino, data_arduino = ardudrv.init_arduino_line()
    ardudrv.send_arduino_cmd_motor(serial_arduino,cmdl,cmdr)

    sync0, timeAcq0, sensLeft0, sensRight0, posLeft0, posRight0 =  encodrv.read_single_packet(debug=True)
    
    t0 = time.time()
    while (time.time()-t0) < duration:
        time.sleep (tloop)

        sync1, timeAcq1, sensLeft1, sensRight1, posLeft1, posRight1 =  encodrv.read_single_packet(debug=True)
        #print sync1, timeAcq1, sensLeft1, sensRight1, posLeft1, posRight1
        print ("dTime",timeAcq0,timeAcq1,timeAcq1-timeAcq0)
        print ("dOdoL",posLeft0,posLeft1,posLeft1-posLeft0)
        print ("dOdoR",posRight0,posRight1,posRight1-posRight0)

	spdL = delat_odo(posLeft1,posLeft0)/8.0/tloop*60.0
	spdR = delat_odo(posRight1,posRight0)/8.0/tloop*60.0

	print ("RPM Left",spdL,"RPM Right",spdR)
        timeAcq0 = timeAcq1
        posRight0 = posRight1
        posLeft0 = posLeft1

