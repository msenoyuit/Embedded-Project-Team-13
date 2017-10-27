import serial
import time
from enum import Enum

start_byte = 43
stop_byte = 45

num_messages_per_avg = 10

# this will store the line
dTime = time.clock()
dTimeAvg = 0
dCount = 0
cTime = time.clock()
cTimeAvg = 0
cCount = 0
lTime = time.clock()
lTimeAvg = 0
lCount = 0
'''wTime = time.clock()
wTimeAvg = 0
wCount = 0'''
messageStartTime = 0

sequenceCount = 0

sendFreqCount = 0

onceError = True

def getPort():
    ports = ['COM%s' % (i + 1) for i in range(256)]
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except (OSError, serial.SerialException):
            pass
    raise Exception("NO PORTS CONNECTED")
	
ser = serial.Serial(
    port=getPort(),\
    baudrate=57600,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
    timeout=1.1)

print("connected to: " + ser.portstr)

def readByte():
    return int.from_bytes(ser.read(1),  byteorder='little')
	
	


def updateStats(readMessage):
    global dTimeAvg
    global dTime
    global dCount
    global cTimeAvg
    global cTime
    global cCount
    global lTimeAvg
    global lTime
    global lCount
    '''global wTimeAvg
    global wTime
    global wCount'''
    if 'Distance' in readMessage:
        dTimeAvg += messageStartTime - dTime
        dTime = messageStartTime
        dCount += 1
        if dCount == num_messages_per_avg:
                print("Distance Sensor Time Avg: ", str(dTimeAvg / 10))
                dTimeAvg = 0
                dCount = 0
        #ser.write(b'\xFFTestString\xFE')
    elif 'Color' in readMessage:
        cTimeAvg += messageStartTime - cTime
        cTime = messageStartTime
        cCount += 1
        if cCount == num_messages_per_avg:
            print("Color Sensor Time Avg: ", str(cTimeAvg / 10))
            cTimeAvg = 0
            cCount = 0
    elif 'Line' in readMessage:
        lTimeAvg += messageStartTime - lTime
        lTime = messageStartTime
        lCount += 1
        if lCount == num_messages_per_avg:
            print("Line Sensor Time Avg: ", str(lTimeAvg / 10))
            lTimeAvg = 0
            lCount = 0
    '''elif 'Wifly' in readMessage:
        wTimeAvg += messageStartTime - wTime
        wTime = messageStartTime
        wCount += 1
        if wCount == num_messages_per_avg:
            print("Line Sensor Time Avg: ", str(lTimeAvg / 10))
            wTimeAvg = 0
            wCount = 0'''

def calculateChecksum(payloadMessage):
    sum = 0
    for char in payloadMessage:
        sum += ord(char)
    return sum & 0xFF

class State(Enum):
    INIT            = 0
    PARSING_MESSAGE = 1

currentState = State.INIT

while True:
    #time.sleep(.0001)
    # Read in the next character
    char = readByte()

    # State machine for reading in message
    if currentState == State.INIT:
        # wait for start character
        if char == start_byte:
            messageStartTime = time.clock()
            message = ''
            currentState = State.PARSING_MESSAGE

    elif currentState == State.PARSING_MESSAGE:
        if char == stop_byte:
            #if("Wifly" in message):
            print(message)
            #    print(sequenceCount)	
            if("ERROR" in message):
                print("")
                print("__________________________ERROR____________________________________________________")
                print("__________________________ERROR____________________________________________________")
                print(message)
                #print("***********************ERROR***********************************************")
                print("")
                #//raise Exception("ERR")
            updateStats(message)
            currentState = State.INIT
            # Send a reply message for each one received
            
            #print(stringToSend)
		
            sendFreqCount += 1
            if sendFreqCount % 10 == 0 and "Wifly" not in message and 1 == 1:
                if(sendFreqCount % 100 == 0 and False):
                    #sequenceCount = int(sequenceCount/2)
                    stringToSend = '0,' + '{0:03}'.format(sequenceCount) + ',10,' + 'TestString,' + '{0:03}'.format(calculateChecksum('TestString'))
                    print("***********************ERROR***********************************************")
                    print("sent: ", stringToSend)
                    print("***********************ERROR***********************************************")
                else:
                    stringToSend = '0,' + '{0:03}'.format(sequenceCount) + ',10,' + 'TestString,' + '{0:03}'.format(calculateChecksum('TestString'))
                ser.write('+'.encode('ascii'))
                ser.write(stringToSend.encode('ascii'))
                ser.write('-'.encode('ascii'))
                sequenceCount += 1
                sequenceCount %= 256
                

        else:
            message += chr(char)

ser.close()
