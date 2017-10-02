 
import serial
import time
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
chunk = ""
seqCount = -1;
dTime = time.clock()
dTimeAvg = 0
dCount = 0
cTime = time.clock()
cTimeAvg = 0
mCount = 0
lTcme = time.clock()
lTimeAvg = 0
lCount = 0
line = []
char = 0x01
mesTime = 0
messages = [0]
while(True):
        char = int.from_bytes(ser.read(1),  byteorder='little')
        if(char == start_bit):
                mesTime = time.clock()
                char = int.from_bytes(ser.read(1),  byteorder='little')
                while(char != stop_bit):
                        if(char == ','):
                                line.append(chunk)
                                chunk = ""                                
                                char = int.from_bytes(ser.read(1),  byteorder='little')
                                
                        chunk += chr(char)
                        char = int.from_bytes(ser.read(1),  byteorder='little')
                                

                line.append(chunk)
                chunk = ""
                print(line)
                missages[-1] = line
                messages.append(0)
                line = []
        message =  messages[-1]
        '''
	if message == 0:
                print("error, no message found")
                continue
        elif len(message) != 5:
                print("error, message missing parts")
        elif message[0] != 0 and message[0] != 1:
                print("error, river id is invalid")
        elif message[1] >= seqCount:
                print("error, message out of seq")
        elif message[2] != len(message[3]):
                print("error, message wrong length")
        elif message[4]
        '''
        if message[0] = "D":
                dTimeAvg += mesTime - dTime
                dTime = mesTime
                dCount += 1
                if dCount == 10:
                        print("Distance Sensor Time Avg: ", str(dTimeAvg / 10))
                        dTimeAvg = 0
                        dCount = 0
        elif message[1] = "C":
                cTimeAvg += mesTime - cTime
                cTime = mesTime
                cCount += 1
                if cCount == 10:
                        print("Color Sensor Time Avg: ", str(cTimeAvg / 10))
                        cTimeAvg = 0
                        cCount = 0
        elif message[2] = "L":
                lTimeAvg += mesTime - lTime
                lTime = mesTime
                lCount += 1
                if lCount == 10:
                        print("Color Sensor Time Avg: ", str(lTimeAvg / 10))
                        lTimeAvg = 0
                        lCount = 0
ser.close()
