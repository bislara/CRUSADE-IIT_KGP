import time
import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)

while True:
	x=ser.readline()
	print(x)
