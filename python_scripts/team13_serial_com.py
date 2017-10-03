import serial
import time
from enum import Enum

start_byte = 0xFF
stop_byte = 0xFE

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

ser = serial.Serial(
    port='COM13',\
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
            print(message)
            updateStats(message)
            currentState = State.INIT
            # Send a reply message for each one received
            stringToSend = '\xFF0,' + '{0:03}'.format(sequenceCount) + ',10,' + 'TestString,' + '{0:03}'.format(calculateChecksum('TestString')) + '\xFE'
            #print(stringToSend)
            ser.write(stringToSend.encode('ascii'))
            sequenceCount += 1
        else:
            message += chr(char)

ser.close()
