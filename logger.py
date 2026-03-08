import datetime
import serial
import struct

port = serial.Serial("COM3", baudrate=921600, stopbits=1, parity=serial.PARITY_NONE)

while True:
    data = port.readline()  # read exactly 24 bytes
    # print(data, len(data))
    try:
        values = list(struct.unpack('<6f', data[:-1]))  # Remove the newline character
        for i, value in enumerate(values):
                values[i] = round(value, 3)
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print(values)
    except:
        pass