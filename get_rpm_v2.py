import encoders_driver_v2 as encdrv
import arduino_driver_v2 as ardudrv
import sys
import time
import numpy as np

def delta_odo (odo1,odo0):
    dodo = odo1-odo0
    if dodo > 32767:
        dodo -= 65536
    if dodo < -32767:
        dodo += 65536
    return dodo


if __name__ == "__main__":
    
    timeout = 1.0
    cmdl = 50
    cmdr = 50
    try:
        cmdl = int(sys.argv[1])
    except:
        pass
    try:
        cmdr = int(sys.argv[2])
    except:
        pass        
    duration = 60.0
    tloop = 2.0 # 0.5 Hz loop
    ard = ardudrv.ArduinoIO()
    ard.send_arduino_cmd_motor (cmdl,cmdr)
    encod_drv = encdrv.EncoderIO()      

    t0 = time.time()
    while (time.time()-t0) < duration:
        t0loop = time.time()

        st0,st1 = encod_drv.get_last_and_older_values_v2()
        data_encoders0 = np.array(st0.split(",")).astype(np.float)
        data_encoders1 = np.array(st1.split(",")).astype(np.float)

        timeAcq0 = data_encoders0[0]/10.0
        sensLeft0 = data_encoders0[1]
        sensRight0 = data_encoders0[2]
        posLeft0 = data_encoders0[3]
        posRight0 = data_encoders0[4]

        timeAcq1 = data_encoders1[0]/10.0
        sensLeft1 = data_encoders1[1]
        sensRight1 = data_encoders1[2]
        posLeft1 = data_encoders1[3]
        posRight1 = data_encoders1[4]

        #print sync1, timeAcq1, sensLeft1, sensRight1, posLeft1, posRight1
        print ("dTime",timeAcq0,timeAcq1,timeAcq1-timeAcq0)
        print ("dOdoL",posLeft0,posLeft1,posLeft1-posLeft0)
        print ("dOdoR",posRight0,posRight1,posRight1-posRight0)

        rpmL = delta_odo(posLeft1,posLeft0)/8.0/tloop*60.0
        rpmR = delta_odo(posRight1,posRight0)/8.0/tloop*60.0
        print ("RPM Left",rpmL,"RPM Right",rpmR)

        while (time.time()-t0loop) < tloop:
            sync1,data_encoders1 = encoddrv.read_packet(debug=False)
            time.sleep(0.001)

    ard.send_arduino_cmd_motor (0,0)
 
