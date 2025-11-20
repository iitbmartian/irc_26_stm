import serial
import time
import numpy as np
n_quad = 1
ser = serial.Serial(
    port='COM9',
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)
try:
    ser.close()
except:
    pass

time.sleep(2)
ser.open()

while True:
    try:
        #ser.write(bytes([1, 2, 3]))
        bytesToRead = ser.inWaiting()
        data = ser.read(bytesToRead)
        if len(data) != 0:
            #print(data) 
            low_byte = np.uint16(data[-1])
            high_byte = np.uint16(data[-2])
            print(np.uint16(low_byte | (high_byte << 8))/100)
    except Exception as e:
        print(e)
    time.sleep(0.1)
ser.close()