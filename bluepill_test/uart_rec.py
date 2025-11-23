import serial
import time
import numpy as np
n_quad = 2
ser = serial.Serial(
    port='COM11',
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
        # print(bytesToRead)
        frame = [np.int32(i) for i in data[-12*n_quad-2:-2]]
        #print(frame)
        #print(data)
        #data_decode = np.array(dtype=np.int32)
        low_byte = np.uint16(data[-1])
        high_byte = np.uint16(data[-2])
        print(np.uint16(low_byte | (high_byte << 8))/100, end = ", ")

        for i in range(0, n_quad):
            #data_decode.append(
            print(np.int32((frame[12*i] & 0xFF) | ((frame[12*i+1] & 0xFF) << 8) | ((frame[12*i+2] & 0xFF) << 16) | ((frame[12*i+3] & 0xFF) << 24)), end= ", ")
            print(np.int32(frame[12*i + 4] & 0xFF) | ((frame[12*i+1+4] & 0xFF) << 8) | ((frame[12*i+2+4] & 0xFF) << 16) | ((frame[12*i+3+4] & 0xFF) << 24), end= ", ")
            print(np.int32((frame[12*i+8] & 0xFF) | ((frame[12*i+1+8] & 0xFF) << 8) | ((frame[12*i+2+8] & 0xFF) << 16) | ((frame[12*i+3+8] & 0xFF) << 24)), end = ", ")
        print()
        #print(data_decode[-1])
    except Exception as e:
        print(e)
    time.sleep(0.1)
ser.close()