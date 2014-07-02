import serial

ser = serial.Serial("/dev/ttyUSB0", 9600)

serin = ser.read()

ser.write(b"1")
