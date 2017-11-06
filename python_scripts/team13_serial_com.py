import serial
from enum import Enum
import threading
import sys

start_byte = 43
stop_byte = 45


class State(Enum):
    INIT = 0
    PARSING_MESSAGE = 1

currentState = State.INIT


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


def readByte():
    return int.from_bytes(ser.read(1),  byteorder='little')


def calculateChecksum(payloadMessage):
    sum = 0
    for char in payloadMessage:
        sum += ord(char)
    return sum & 0xFF


sequenceCount = 0


def sendMessage(message):
    global sequenceCount
    stringToSend = '0,{0:03},{1:02},{2},{3:03}'.format(
        sequenceCount, len(message), message, calculateChecksum('TestString'))
    ser.write('+'.encode('ascii'))
    ser.write(stringToSend.encode('ascii'))
    ser.write('-'.encode('ascii'))
    sequenceCount += 1
    sequenceCount %= 256


def processRead(char):
    global currentState
    # State machine for reading in message
    if currentState == State.INIT:
        # wait for start character
        if char == start_byte:
            message = ''
            currentState = State.PARSING_MESSAGE

    elif currentState == State.PARSING_MESSAGE:
        if char == stop_byte:
            print(message)
            currentState = State.INIT
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
