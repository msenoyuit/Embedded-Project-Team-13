from roverControl import RoverControl
from PyQt5.QtCore import *
import commandDefs
import time

class Rover0Control(RoverControl):

    # Signals                    index,x,y
    newRedBlockFound = pyqtSignal(int,int,int)
    newGreenBlockFound = pyqtSignal(int, int, int)
    newBlueBlockFound = pyqtSignal(int, int, int)

    def __init__(self, positionInfo):
        super().__init__(positionInfo)
        # Indices for keeping track of which blocks have been found
        self.redBlockIndex = 0
        self.greenBlockIndex = 0
        self.blueBlockIndex = 0

    def getCurrentPosition(self):
        return [self.positionInfo.rover0X, self.positionInfo.rover0Y]

    def updateCurrentPosition(self, x, y):
        self.positionInfo.setRover0Pos(x, y)
        self.positionChanged.emit(0, x, y)

    def updateNextPosition(self, x, y):
        self.positionInfo.setNextRover0Pos(x, y)

    def waitUntilClear(self):#, x, y):
        self.positionInfo.waitUntilRover0Clear()
        
    def distanceColor(self,r,g,b):
        if r - 50 > g  and r - 50 > b:
            return "R"
        if g > b:
            return 'G'
        return 'B'
        # Command a color reading and return the result
    def takeColorReading(self):
        time.sleep(0.5)
        readCommand = commandDefs.commands["READ_COMMAND"] + ' ' + '0'
        self.sendCommand(readCommand)
        nextMessage = self.messageQ.get().split(' ')
        while len(nextMessage) != 6:
            self.infoMessage('Unexpected message from rover during takeColorReading:\t' + nextMessage)
            nextMessage = self.messageQ.get().split(' ')
        '''while len(nextMessage) != 2 or nextMessage[0] != commandDefs.flags["EVENT_ALERT"]:
            self.infoMessage('Unexpected message from rover during takeColorReading:\t' + nextMessage)
            nextMessage = self.messageQ.get().split(' ')'''
        toReturn = self.distanceColor(int(nextMessage[1]), int(nextMessage[3]), int(nextMessage[5])) # for x in [nextMessage[1], nextMessage[3], nextMessage[5]]
        self.waitForCommandFinished(readCommand)
        return toReturn

    def nextStepAlongRow(self, direction):
        if self.moveCommand(direction) == commandDefs.flags["EVENT_ALERT"]:  # When a block is encountered
            blockColor = self.takeColorReading()  # Take a color reading
            if blockColor == 'R':
                self.newRedBlockFound.emit(self.redBlockIndex, self.positionInfo.nextRover0X,
                                           self.positionInfo.nextRover0Y)
                self.redBlockIndex += 1
            elif blockColor == 'G':
                self.newGreenBlockFound.emit(self.greenBlockIndex, self.positionInfo.nextRover0X,
                                             self.positionInfo.nextRover0Y)
                self.greenBlockIndex += 1
            elif blockColor == 'B':
                self.newBlueBlockFound.emit(self.blueBlockIndex, self.positionInfo.nextRover0X,
                                            self.positionInfo.nextRover0Y)
                self.blueBlockIndex += 1
            self.moveAroundBlock(direction)

    def run(self):
        while self.getCurrentPosition()[1] < 8:
            while self.getCurrentPosition()[0] < 8:
                self.nextStepAlongRow("EAST_MOVE")
            self.moveCommand("NORTH_MOVE")
            while self.getCurrentPosition()[0] > 0:
                self.nextStepAlongRow("WEST_MOVE")
            self.moveCommand("NORTH_MOVE")
        while self.getCurrentPosition()[0] < 8:
            self.nextStepAlongRow("EAST_MOVE")