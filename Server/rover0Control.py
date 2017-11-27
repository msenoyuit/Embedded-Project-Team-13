from roverControl import RoverControl
from PyQt5.QtCore import *
import commandDefs

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

    # Command a color reading and return the result
    def takeColorReading(self):
        self.sendCommand(commandDefs.commands["READ_COMMAND"])
        nextMessage = self.messageQ.get().split(' ')
        while len(nextMessage) != 2 or nextMessage[0] != commandDefs.flags["EVENT_ALERT"]:
            self.infoMessage('Unexpected message from rover during takeColorReading:\t' + nextMessage)
            nextMessage = self.messageQ.get().split(' ')
        self.waitForCommandFinished(commandDefs.commands["READ_COMMAND"])
        return nextMessage[1]

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