#!/usr/bin/env python3
import serial
from enum import Enum
import threading
import sys

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
commandList = ['3 5 1', '3 6 1', '3 7 1', '4 5 1', '4 6 1','3 5 1','4 7 1', '0 1 1']
messageStartTime = 0


class State(Enum):
    INIT = 0
    PARSING_MESSAGE = 1

commandNumber = 0

onceError = True


def calculateChecksum(payloadMessage):
    sum = 0
    for char in payloadMessage:
        sum += ord(char)
    return sum & 0xFF


def formatString(inString):
    idNumber = '0'
    strLength = '{0:02}'.format(len(inString))
    seqCount = '{0:03}'.format(sequenceCount)
    checkSum = '{0:03}'.format(calculateChecksum(inString))
    stringToSend = idNumber + ',' + seqCount + ',' + strLength + ',' + inString + ',' + checkSum
    return stringToSend


def getPort():
    return "/dev/tty.usbserial-A702ZMVF"  # For osx!
    ports = ['COM%s' % (i + 1) for i in range(256)]
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except (OSError, serial.SerialException):
            pass
    raise Exception("NO PORTS CONNECTED")

ser = serial.Serial(port=getPort(), baudrate=57600, parity=serial.PARITY_NONE,
                    stopbits=serial. STOPBITS_ONE, bytesize=serial.EIGHTBITS,
                    timeout=1.1)

print("connected to: " + ser.portstr)
stringToSend = formatString("3 5 1")
#ser.write('+'.encode('ascii'))
#ser.write(stringToSend.encode('ascii'))
#ser.write('-'.encode('ascii'))
#sequenceCount += 1


def readByte():
    return int.from_bytes(ser.read(1),  byteorder='little')


sequenceCount = 0


def sendMessage(message):
    global sequenceCount
    message = message.strip()
    if (message == 'reset'):
        sequenceCount = 0
        return
    stringToSend = '0,{0:03},{1:02},{2},{3:03}'.format(
        sequenceCount, len(message), message, calculateChecksum(message))
    ser.write('+'.encode('ascii'))
    ser.write(stringToSend.encode('ascii'))
    ser.write('-'.encode('ascii'))
    sequenceCount += 1
    sequenceCount %= 256


def processRead(char):
    global currentState
    global message
    # State machine for reading in message
    if currentState == State.INIT:
        # wait for start character
        if char == start_byte:
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
                    #stringToSend = '0,' + '{0:03}'.format(sequenceCount) + ',05,' + '3 5 1,' + '{0:03}'.format(calculateChecksum('3 5 1'))
                    commandIn = commandList[commandNumber]
                    commandNumber += 1
                    commandNumber = commandNumber % len(commandList)
                    stringToSend = formatString(commandIn)
                    print("*******************"+ stringToSend + "******************")
                ser.write('+'.encode('ascii'))
                ser.write(stringToSend.encode('ascii'))
                ser.write('-'.encode('ascii'))
                sequenceCount += 1
                sequenceCount %= 256
                
        else:
            message += chr(char)


def sendthreadfn():
    for line in sys.stdin:
        sendMessage(line)


if __name__ == '__main__':
    thread = threading.Thread(target=sendthreadfn)
    thread.start()
    while True:
        # Read in the next character
        processRead(readByte())

    ser.close()
