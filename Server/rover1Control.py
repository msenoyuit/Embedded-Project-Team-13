from roverControl import RoverControl
from PyQt5.QtCore import *
import time
import commandDefs

class Rover1Control(RoverControl):

    # Signals
    pickedUpBlock = pyqtSignal(int) # index
    #                   index,newX,newY
    movedBlock = pyqtSignal(int,int,int)

    def __init__(self, positionInfo):
        super().__init__(positionInfo)

        # Info for tracking where the blocks are
        self.redBlocks = [[-1,-1],[-1,-1],[-1,-1]]
        self.greenBlocks = [[-1,-1],[-1,-1],[-1,-1]]
        self.blueBlocks = [[-1,-1],[-1,-1],[-1,-1]]

        self.hasBlock = None

    def getCurrentPosition(self):
        return [self.positionInfo.rover1X, self.positionInfo.rover1Y]

    def updateCurrentPosition(self, x, y):
        self.positionInfo.setRover1Pos(x, y)
        self.positionChanged.emit(1, x, y)
        if self.hasBlock != None:
            self.movedBlock.emit(self.hasBlock, x, y)

    def updateNextPosition(self, x, y):
        self.positionInfo.setNextRover1Pos(x, y)

    def waitUntilClear(self):
        self.positionInfo.waitUntilRover1Clear()

    def addRedBlock(self, index, x, y):
        self.redBlocks[index] = [x, y]

    def addGreenBlock(self, index, x, y):
        self.greenBlocks[index] = [x, y]

    def addBlueBlock(self, index, x, y):
        self.blueBlocks[index] = [x, y]

    # Return the color of the block if present, None otherwise
    def isBlockAtPosition(self, x, y):
        if [x, y] in self.redBlocks:
            return 'R'
        if [x, y] in self.greenBlocks:
            return 'G'
        if [x, y] in self.blueBlocks:
            return 'B'
        return None

    def isBlockInDirection(self, direction):
        [x, y] = self.getCurrentPosition()
        if direction == "NORTH_MOVE":
            return self.isBlockAtPosition(x, y+1)
        if direction == "EAST_MOVE":
            return self.isBlockAtPosition(x+1, y)
        if direction == "SOUTH_MOVE":
            return self.isBlockAtPosition(x, y-1)
        if direction == "WEST_MOVE":
            return self.isBlockAtPosition(x-1, y)

    def waitForBlockToBeFound(self, index):
        while self.redBlocks[index] == [-1,-1]:
            time.sleep(0.1)

    def run(self):
        self.moveCommand("EAST_MOVE") # Move to the 0 position
        for blockIndex in range(0,3):
            self.waitForBlockToBeFound(blockIndex)
            while self.getCurrentPosition()[1] < self.redBlocks[blockIndex][1]: # Move up while y < block's y
                self.moveCommand("NORTH_MOVE")
            while self.getCurrentPosition()[0] < self.redBlocks[blockIndex][0] - 1: # Move right while left of block
                if not self.isBlockInDirection("EAST_MOVE"):
                    self.moveCommand("EAST_MOVE")
                else:
                    self.moveAroundBlock("EAST_MOVE")
            pickupCommand = commandDefs.commands["PICKUP_COMMAND"] + ' ' + commandDefs.specifiers["EAST_MOVE"]
            self.sendCommand(pickupCommand)                                     # Pick up the block
            self.waitForCommandFinished(pickupCommand)
            self.hasBlock = blockIndex
            self.updateCurrentPositionByDirection("EAST_MOVE")
            self.pickedUpBlock.emit(blockIndex)
            while self.getCurrentPosition()[0] > 0:                             # Move left while right of axis
                if not self.isBlockInDirection("WEST_MOVE"):
                    self.moveCommand("WEST_MOVE")
                else:
                    self.moveAroundBlock("WEST_MOVE")
            while self.getCurrentPosition()[1] > 0:                             # Move down while above bottom
                self.moveCommand("SOUTH_MOVE")
            self.sendCommand(commandDefs.commands["RELEASE_COMMAND"])           # Release block
            self.waitForCommandFinished(commandDefs.commands["RELEASE_COMMAND"])
            self.hasBlock = None


