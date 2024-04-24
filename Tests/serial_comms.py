import serial, time

#ser = serial.Serial('/dev/ttyUSB0')
#with serial.Serial() as ser:
ser = serial.Serial()    
ser.baudrate = 9600
ser.port = '/dev/ttyUSB0'
#ser.open()
    #ser.write(b'hello')
#print(ser.name)
while(True):
	ser.open()
	#ser = serial.Serial('/dev/ttyUSB0')
	print("Sending..")
	ser.write(b'hello')
	#ser.close() 
	#ser = serial.Serial('/dev/ttyUSB0')
	#line = ser.readline()	
	ser.close() 
	time.sleep(2)
