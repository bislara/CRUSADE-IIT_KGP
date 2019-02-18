import serial
import struct
import time


ser = serial.Serial('/dev/ttyUSB1',9600)
# time.sleep(0.2)
while(cv2.waitKey(25) & 0xFF != ord('q')):
    ser.write('L')
