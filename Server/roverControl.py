from PyQt5.QtCore import *
import queue
import commandDefs

class RoverControl(QObject):

    # Signals
    sendCommandSignal = pyqtSignal(str)
    #                           rvrid,newx,newy
    positionChanged = pyqtSignal(int, int, int)

    def __init__(self, positionInfo):
        super().__init__()
        self.positionInfo = positionInfo
        self.messageQ = queue.Queue()

    def getCurrentPosition(self):
        print("WARNING: Subclass of RoverControl must implement getCurrentPosition")
        return [0,0]

    def updateCurrentPosition(self, x, y):
        print("WARNING: Subclass of RoverControl must implement updateCurrentPosition")

    def updateCurrentPositionByDirection(self, direction):
        [x, y] = self.getCurrentPosition()
        if direction == "NORTH_MOVE":
            self.updateCurrentPosition(x, y+1)
        elif direction == "EAST_MOVE":
            self.updateCurrentPosition(x+1, y)
        elif direction == "SOUTH_MOVE":
            self.updateCurrentPosition(x, y-1)
        elif direction == "WEST_MOVE":
            self.updateCurrentPosition(x-1, y)

    def updateNextPosition(self, x, y):
        print("WARNING: Subclass of RoverControl must implement updateNextPosition")

    def updateNextPositionByDirection(self, direction):
        [x, y] = self.getCurrentPosition()
        if direction == "NORTH_MOVE":
            self.updateNextPosition(x, y + 1)
        elif direction == "EAST_MOVE":
            self.updateNextPosition(x + 1, y)
        elif direction == "SOUTH_MOVE":
            self.updateNextPosition(x, y - 1)
        elif direction == "WEST_MOVE":
            self.updateNextPosition(x - 1, y)

    def waitUntilClear(self):#, x, y):
        print("WARNING: Subclass of RoverControl must implement waitUntilClear")

    # Send a command and wait until we get an ack
    def sendCommand(self, command):
        self.sendCommandSignal.emit(command)
        expectedReply = commandDefs.flags["COMMAND_RECEIVED"] + ' ' + command
        nextReply = self.messageQ.get()
        while nextReply != expectedReply:
            self.infoMessage('Unexpected message from rover during sendCommand:\t' + nextReply)
            nextReply = self.messageQ.get()

    def waitForCommandFinished(self, command):
        reply = self.messageQ.get()
        while reply != commandDefs.flags["COMMAND_FINISHED"] + ' ' + command:
            self.infoMessage('Unexpected message while waiting for command completion:\t' + reply)
            reply = self.messageQ.get()

    # Attempt a move command and return the result; update position if successful
    def moveCommand(self, direction):
        self.updateNextPositionByDirection(direction)
        self.waitUntilClear()
        command = commandDefs.commands["MOVE_COMMAND"] + ' ' + commandDefs.specifiers[direction]
        self.sendCommand(command)
        roverReplyFlag = self.messageQ.get().replace(' ' + command, '')
        while True:
            if roverReplyFlag == commandDefs.flags["EVENT_ALERT"]:
                return roverReplyFlag
            if roverReplyFlag == commandDefs.flags["COMMAND_FINISHED"]:
                self.updateCurrentPositionByDirection(direction)
                return roverReplyFlag
            self.infoMessage('Unexpected message from rover during moveCommand:\t')
            roverReplyFlag = self.messageQ.get().replace(' ' + command, '')

    def moveAroundBlock(self, directionOfTravel):
        if self.getCurrentPosition()[1] == 0:   # if on the bottom row
            self.moveCommand("NORTH_MOVE")
            self.moveCommand(directionOfTravel)
            self.moveCommand(directionOfTravel)
            self.moveCommand("SOUTH_MOVE")
        else:
            self.moveCommand("SOUTH_MOVE")
            self.moveCommand(directionOfTravel)
            self.moveCommand(directionOfTravel)
            self.moveCommand("NORTH_MOVE")

    def infoMessage(self, message):
        print(message)

    def errorMessage(self, message):
        print(message)

    # Handle a message from the rover
    def handleMessage(self, message):
        self.messageQ.put(message)
