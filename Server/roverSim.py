from testClient import TestClient
from messageparser import MessageParser
import commandDefs
from PyQt5.QtWidgets import QApplication
import sys
import time

class RoverSim():

    def __init__(self, roverID, blockInfo):
        self.roverID = roverID
        self.blockInfo = blockInfo
        self.posY = 0
        self.nextY = 0
        if roverID == 0:
            self.posX = 0
            self.nextX = 0
        else:
            self.posX = -1
            self.nextX = -1

        # Declare test client and parser
        self.clientConnection = TestClient()
        self.parser = MessageParser(roverID)

        # Connect signals and slots
        self.clientConnection.messageFromServer.connect(self.parser.parse)
        self.parser.parsedMessageSignal.connect(self.messageFromServer)
        self.parser.constructedMessageSignal.connect(self.clientConnection.sendMessageToServer)

    def ack(self, command):
        self.parser.constructMessage(commandDefs.flags["COMMAND_RECEIVED"] + ' ' + command)

    def finish(self, command):
        time.sleep(0.5) # Pause for command to complete
        self.parser.constructMessage(commandDefs.flags["COMMAND_FINISHED"] + ' ' + command)

    def eventAlert(self):
        self.parser.constructMessage(commandDefs.flags["EVENT_ALERT"])

    # Handler for server commands
    def messageFromServer(self, message):
        # Acknowledge message
        self.ack(message)

        messageFields = message.split(' ')
        # Compute new position if necessary
        if messageFields[0] == commandDefs.commands["MOVE_COMMAND"] or messageFields[0] == commandDefs.commands["PICKUP_COMMAND"]:
            if messageFields[1] == commandDefs.specifiers["NORTH_MOVE"]:
                self.nextX = self.posX
                self.nextY = self.posY + 1
                self.executeMove(message)
            elif messageFields[1] == commandDefs.specifiers["EAST_MOVE"]:
                self.nextX = self.posX + 1
                self.nextY = self.posY
                self.executeMove(message)
            elif messageFields[1] == commandDefs.specifiers["SOUTH_MOVE"]:
                self.nextX = self.posX
                self.nextY = self.posY - 1
                self.executeMove(message)
            elif messageFields[1] == commandDefs.specifiers["WEST_MOVE"]:
                self.nextX = self.posX - 1
                self.nextY = self.posY
                self.executeMove(message)
        # Send reading for a read command
        elif messageFields[0] == commandDefs.commands["READ_COMMAND"]:
            self.parser.constructMessage(commandDefs.flags["EVENT_ALERT"] + ' ' + self.blockInfo[8-self.nextY][self.nextX])
            self.finish(message)
        # Finish any other command
        else:
            self.finish(message)

    def blockAtPos(self, x, y):
        if self.blockInfo[8-y][x] != 'N':
            return self.blockInfo[8-y][x]
        else:
            return None

    def updatePos(self):
        self.posX = self.nextX
        self.posY = self.nextY
        print("New positiion: ", self.posX, ', ', self.posY)

    def executeMove(self, message):
        if self.roverID == 0:
            if not self.blockAtPos(self.nextX, self.nextY): # if no block is in the way
                self.updatePos()                            # update position and finish command
                self.finish(message)
            else:                                           # if a block is in the way
                self.eventAlert()                           # send an event alert
        else:
            self.updatePos()                                # Moves always succeed for rover1
            self.finish(message)

    def run(self):
        app = QApplication(sys.argv)
        self.clientConnection.begin.emit()
        sys.exit(app.exec())