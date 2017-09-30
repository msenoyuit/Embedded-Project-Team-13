
import serial
start_bit = 0xFF
stop_bit = 0xFE
ser = serial.Serial(
    port='COM6',\
    baudrate=57600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=1.1)

print("connected to: " + ser.portstr)

#this will store the line
line = ""
char = 0x01
while(True):
        char = int.from_bytes(ser.read(1),  byteorder='little')
        if(char == start_bit):
                char = int.from_bytes(ser.read(1),  byteorder='little')
                while(char != stop_bit):
                        line += chr(char)
                        char = int.from_bytes(ser.read(1),  byteorder='little')
                print(line)
                line = ""
	
ser.close()
