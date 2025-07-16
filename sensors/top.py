import serial
import time
ser = serial.Serial("/dev/ttyAMA1", 115200, timeout=1)

def get_lidar_distance():
    count = ser.in_waiting
    if count > 8:
        recv = ser.read(9)
        if recv[0] == 0x59 and recv[1] == 0x59:
            return recv[2] + recv[3] * 256
    return float('inf')  
