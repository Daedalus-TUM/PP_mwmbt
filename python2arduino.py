import serial
import time

ser = serial.Serial("/dev/ttyUSB0", 9600)
time.sleep(2)

ser.write(bytes([10]))
ser.write(bytes([20]))
ser.write(bytes([30]))

ser.close()
