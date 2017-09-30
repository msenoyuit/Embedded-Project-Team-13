import serial

ser = serial.Serial(
    port='COM5',\
    baudrate=57600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=1.1)

print("connected to: " + ser.portstr)

#this will store the line
line = []
while(True):
	mess = ser.read(1)
	ser.write(mess)
	print(mess)
	
ser.close()