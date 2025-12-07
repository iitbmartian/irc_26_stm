import serial
import time
import numpy as np

n_quad = 5
n_enc = 1
n_acs = 9
n_motors = 2 #13
#8*explicit + 2*drill + 1*Linear base + 2*BLDC 

quad_data = []
buf_size = 12*(n_quad+1) + 2*n_enc + 2*n_acs


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

#UART frame to Nucleo
#PWM1-8 (wheel-expl-wheel-expl) PWM9,10 (Drill) PWM 11 (Lin. base) PWM 12, 13 (BLDC)
#Each frame: LSB=dir, [8:1] = PWM (8 bit) -- not possible
#Odd frame: Dir, Even frame: pwm 8 bit
def send_uart_data(dir_arr, pwm_arr):
    outstring = b""
    for i in range(n_motors):
        outstring += dir_arr[i].to_bytes(1)
        outstring += min(int(pwm_arr[i]), 255).to_bytes(1)
    ser.write(outstring)

# def get_uart_data():
#     #ser.write(bytes([1, 2, 3]))
#     bytesToRead = ser.inWaiting()
#     print(bytesToRead)
#     data = ser.read(bytesToRead)
#     # print(bytesToRead)
#     frame = [np.int32(i) for i in data[0:24]]
#     #print(frame)
#     #print(data)
#     #data_decode = np.array(dtype=np.int32)
#     low_byte = np.uint16(data[25])
#     high_byte = np.uint16(data[24])
#     acsangle = np.uint16(low_byte | (high_byte << 8))/100
#     # print(acsangle)
#     quad_data = [(acsangle)]
#     for i in range(0, n_quad):
#         pos = np.int32((frame[12*i] & 0xFF) | ((frame[12*i+1] & 0xFF) << 8) | ((frame[12*i+2] & 0xFF) << 16) | ((frame[12*i+3] & 0xFF) << 24))
#         diff = np.int32(frame[12*i + 4] & 0xFF) | ((frame[12*i+1+4] & 0xFF) << 8) | ((frame[12*i+2+4] & 0xFF) << 16) | ((frame[12*i+3+4] & 0xFF) << 24)
#         diff2 = np.int32((frame[12*i+8] & 0xFF) | ((frame[12*i+1+8] & 0xFF) << 8) | ((frame[12*i+2+8] & 0xFF) << 16) | ((frame[12*i+3+8] & 0xFF) << 24))
#         quad_data.append((pos, diff, diff2))
#     return(quad_data)

    
def get_uart_data():
    bytesToRead = ser.inWaiting()
    polled_frame = {
        "quadrature": [],
        "AS5600":[],
        "ACS":[]
    }
    print(bytesToRead)
    data = ser.read(bytesToRead)
    last_nl = data.rfind(b'\n')
    print(last_nl)
    data = data[last_nl - buf_size:last_nl] #if last_nl != -1 else data[-buf_size- 1:-1]
    print(data, len(data))
    for i in range(n_quad+1):
        base = 12 * i
        pos = int.from_bytes(data[base:base + 4], byteorder='big', signed=True)
        diff = int.from_bytes(data[base + 4:base + 8], byteorder='big', signed=True)
        diff2 = int.from_bytes(data[base + 8:base + 12], byteorder='big', signed=True)
        # print(np.int32(pos), np.int32(diff), np.int32(diff2))
        polled_frame["quadrature"].append(((np.int32(pos), np.int32(diff), np.int32(diff2))))
    for i in range(n_enc):
        base = 12*(n_quad+1) + 2*i
        low_byte = np.uint16(data[base+1])
        high_byte = np.uint16(data[base])
        # print(np.uint16(low_byte | (high_byte << 8)))
        polled_frame["AS5600"].append(np.uint16(low_byte | (high_byte << 8))/100)
    for i in range(n_acs):
        base = 12*(n_quad+1) + 2*n_enc + 2*i
        low_byte = np.uint16(data[base+1])
        high_byte = np.uint16(data[base])
        # print(np.uint16(low_byte | (high_byte << 8))/100)
        polled_frame["ACS"].append(np.uint16(low_byte | (high_byte << 8)))
    ser.reset_input_buffer()
    # print(polled_frame)
    
    return polled_frame

#if __name__ == "main":
i = 0
kp = 0.5
pwmout = 0
enc_data = {
    "quadrature": [(0,0,0)]*n_quad,
    "AS5600": [0]*n_enc,
    "ACS": [0]*n_acs
}
while True:
    #send_uart_data([i%2], [(i*5)%255])
    try:
        enc_data = get_uart_data()
        for entry in enc_data:
            print(entry, enc_data[entry])
    except Exception as e:
        print(e)
    print(enc_data["quadrature"])
    pwmout += kp*(80 - enc_data["quadrature"][1][1])
    
    dir = int(pwmout < 0)
    pwmout = abs(pwmout)
    print(dir, pwmout)
    send_uart_data([dir, 0], [pwmout, 0])

    time.sleep(1)
    
    i += 1
ser.close()