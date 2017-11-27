import socket
import time

hostname = 'localhost'
portNumber = 50000
maxMessageSize = 1024
sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # Construct socket object
sckt.connect((hostname, portNumber))                        # Connect to server

def sendString(stringToSend):
    stringBytes = bytearray([ ord(x) for x in stringToSend ])
    sckt.send(stringBytes)
    #reply = sckt.recv(maxMessageSize)
    #print('Server reply:\t', ''.join([chr(x) for x in reply]))

def receiveReply():
    reply = sckt.recv(maxMessageSize)
    replyString = ''.join([chr(x) for x in reply])
    return replyString

def test_valid_messages():
    # Send two valid messages and verify that the server confirms successful receipt

    sendString("\xFF0,000,011,TestString1,072\xFE")
    reply = receiveReply()
    assert reply == '\xFF0,000,029,Received Message:\tTestString1,151\xFE'

    sendString("\xFF0,001,011,TestString2,073\xFE")
    reply = receiveReply()
    assert reply == '\xFF0,001,029,Received Message:\tTestString2,152\xFE'

def test_invalid_messages():
    # Send a series of erroneous messages and verify the results in the server log

    # Message doesn't begin with start byte
    sendString('A')
    # Message has invalid rover ID
    sendString("\xFF3,002,011,TestString3,074\xFE")
    # Incorrect sequence count
    sendString("\xFF0,999,011,TestString3,074\xFE")
    # Incorrect message length
    sendString("\xFF0,002,000,TestString3,074\xFE")
    # Message without stop bit
    sendString("\xFFPartial Message\xFF0,002,011,TestString3,074\xFE")

    # Verify last valid message was accepted
    reply = receiveReply()
    assert reply == '\xFF0,002,029,Received Message:\tTestString3,153\xFE'

    # Close socket and wait for server to close log
    sckt.close()
    time.sleep(1)

    # Open server log and verify contents
    serverLog = open('ServerLog.txt', 'r')
    logFileLines = serverLog.readlines()

    assert logFileLines[2] == 'Parse Info:\tNon-start byte while in init state:\t' + str(ord('A')) + '\n'
    assert logFileLines[3] == 'Parse Error:\tRover ID incorrect\n'
    assert logFileLines[4] == 'Parse Error:\tIncorrect sequence count\n'
    assert logFileLines[5] == 'Parse Error:\tIndicated message length doesn\'t match actual message length\n'
    assert logFileLines[6] == 'Parse Error:\tMessage did not contain a stop byte:\tÿPartial Message\n'


