import serial

ser = serial.Serial('/dev/ttyUSB0')
print(ser.name)
ser.write('hello')
line = ser.readline()
ser.close() 