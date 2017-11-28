import checksum
from PyQt5.QtCore import *

class MessageParser(QObject):

    startByte = 0xFF
    stopByte = 0xFE

    # Signal emitted when a full message is parsed
    parsedMessageSignal = pyqtSignal(str)
    # Signal emitted with a full message ready to be sent
    constructedMessageSignal = pyqtSignal(str)
    
    def __init__(self, roverIdentifier):
        super().__init__()
        self.receivedMessage = ''
        self.state = 'INIT'
        self.rxSequenceCount = 0
        self.txSequenceCount = 0
        self.roverID = roverIdentifier
        self.logFile = open('ServerLog.txt', 'w')

    def parseError(self, message):
        print('Parse Error:\t' + message)
        self.logFile.write('Parse Error:\t' + message + '\n')

    def parseInfo(self, message):
        print('Parse Info:\t' + message)
        self.logFile.write('Parse Info:\t' + message + '\n')

    def extractPayload(self, message):
        messageFields = message.split(',')
        if not len(messageFields) == 5:
            self.parseError('Message did not contain the proper number of fields')
        elif not (len(messageFields[0]) == 1 and ord(messageFields[0]) - 0x30 == self.roverID):
            self.parseError('Rover ID incorrect')
        elif not (len(messageFields[1]) == 3 and messageFields[1].isdigit()):
            self.parseError('Sequence count malformed')
        elif int(messageFields[1]) != self.rxSequenceCount:
            self.parseError('Incorrect sequence count')
        elif not (len(messageFields[2]) == 3 and messageFields[2].isdigit()):
            self.parseError('Invalid message length')
        elif len(messageFields[3]) != int(messageFields[2]):
            self.parseError('Indicated message length doesn\'t match actual message length')
        elif not (len(messageFields[4]) == 3 and messageFields[4].isdigit()):
            self.parseError('Checksum field malformed')
        elif int(messageFields[4]) != checksum.calculateChecksum(messageFields[3]):
            self.parseError('Checksum mismatch')
        else:
            self.rxSequenceCount += 1
            return messageFields[3]
        return None

    def constructMessage(self, message):
        fullMessage = str(chr(MessageParser.startByte)) + str(self.roverID) + ',' + '{:03}'.format(self.txSequenceCount) + ',' + '{:03}'.format(len(message)) + ','
        fullMessage +=  message + ',' + '{:03}'.format(checksum.calculateChecksum(message)) + chr(MessageParser.stopByte)
        self.txSequenceCount += 1
        #return fullMessage
        self.constructedMessageSignal.emit(fullMessage) # Emit signal with contructed message

    def handleMessage(self):
        messagePayload = self.extractPayload(self.receivedMessage[1:len(self.receivedMessage) - 1]) # Strip start and stop bits
        if messagePayload:
            self.parseInfo('Received Message:\t' + messagePayload)
            self.receivedMessage = ''
            #self.sendFunc(self.constructMessage('Received Message:\t' + messagePayload))
            self.parsedMessageSignal.emit(messagePayload)   # Emit signal with parsed message

    def parse(self, messageChunk):
        for char in messageChunk:
            if self.state == 'INIT':
                if char == MessageParser.startByte:
                    self.receivedMessage = chr(char)
                    self.state = 'PARSE'
                else:
                    self.parseInfo('Non-start byte while in init state:\t' + str(char))
            elif self.state == 'PARSE':
                if char == MessageParser.stopByte:
                    self.receivedMessage += chr(char)
                    self.handleMessage()
                    self.state = 'INIT'
                elif char == MessageParser.startByte:
                    self.parseError('Message did not contain a stop byte:\t' + self.receivedMessage)
                    self.receivedMessage = chr(char)
                else:
                    self.receivedMessage += chr(char)
                    
        #print('Server received:')
        #print(messageChunk)

    def closeLog(self):
        self.logFile.close()
