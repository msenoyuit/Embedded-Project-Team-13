from serverGui import MainWindow
from server import team13Server
from messageparser import MessageParser
from roverPositions import RoverPositions
from rover0Control import Rover0Control
from rover1Control import Rover1Control
import commandDefs
import sys
import os
import threading
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import *

state = 'INIT'
stateToResume = '' # Used to recover from pause
rover0CommandToResume = ''
rover1CommandToResume = ''

# Determine which parser to use based on rover ID
def guiCommandHandler(rover, message):
    if rover == 'ROVER_0':
        rover0Parser.constructMessage(message)
    else:
        rover1Parser.constructMessage(message)

# Called when user presses start button
def start():
    global state, stateToResume, rover0CommandToResume, rover1CommandToResume
    if state == 'INIT': # When first starting, create threads for rover controllers and start them running
        controller0Thread = threading.Thread(target=rover0Controller.run)
        controller0Thread.start()
        controller1Thread = threading.Thread(target=rover1Controller.run)
        controller1Thread.start()
        state = 'RUN'
    if state == 'PAUSE':
        state = stateToResume
        if len(rover0CommandToResume) > 0:
            rover0Parser.constructMessage(rover0CommandToResume)
        if len(rover1CommandToResume) > 0:
            rover1Parser.constructMessage(rover1CommandToResume)
    else:
        mainInfo('Already running (state = ' + state + ')')

def pause():
    global state, stateToResume
    if state == 'RUN':
        state = 'PAUSE'
        stateToResume = 'RUN'
    else:
        mainInfo('Not running')

def reset():
    global state
    state = 'INIT'
    # TODO: complete
    os.system('sudo reboot')

def rover0Message(message):
    global state, rover0Controller
    if ':' in message:
        print('Sensor reading:\t', message)
    elif state != 'INIT':
        rover0Controller.handleMessage(message)
    else:
        print('Dropped rover 0 message:\t', message)

    #rover0Parser.constructMessage('Received Message:\t' + message)
    # TODO: complete

def rover1Message(message):
    global state, rover1Controller
    if ':' in message:
        print('Sensor reading:\t', message)
    elif state != 'INIT':
        rover1Controller.handleMessage(message)
    else:
        print('Dropped rover 1 message:\t', message)

def rover0Command(message):
    global state, rover0Parser, rover0CommandToResume
    if state == 'RUN':
        rover0Parser.constructMessage(message)
    elif state == 'PAUSE':
        rover0CommandToResume = message

def rover1Command(message):
    global state, rover1Parser, rover1CommandToResume
    if state == 'RUN':
        rover1Parser.constructMessage(message)
    elif state == 'PAUSE':
        rover1CommandToResume = message

def mainInfo(message):
    print('Main Info:\t', message)

def mainError(message):
    print('Main Error:\t', message)

def cleanup():
    print('cleanup')
    server.closeServer()

# Make Qt happy
app = QApplication(sys.argv)

# Declare server
server = team13Server('192.168.42.1', 2000)
# Declare server GUI
serverGui = MainWindow()
# Declare parser for rover 0
rover0Parser = MessageParser(0)
# Declare parser for rover 1
rover1Parser = MessageParser(1)
# Central class holding positions of both rovers
roverPositions = RoverPositions()
# Declare controller for rover 0
rover0Controller = Rover0Control(roverPositions)
# Declare controller for rover 1
rover1Controller = Rover1Control(roverPositions)

# Connect server signals
server.client0Message.connect(rover0Parser.parse)
server.client1Message.connect(rover1Parser.parse)
server.serverClosed.connect(rover0Parser.closeLog)
server.startSelf.connect(server.runServer)
# Connect GUI signals
serverGui.guiCommand.connect(guiCommandHandler)
serverGui.startSignal.connect(start)
serverGui.pauseSignal.connect(pause)
serverGui.resetSignal.connect(reset)
serverGui.windowClosed.connect(cleanup)
# Connect parser signals
rover0Parser.parsedMessageSignal.connect(rover0Message)
rover0Parser.constructedMessageSignal.connect(server.sendClient0Message)
rover1Parser.parsedMessageSignal.connect(rover1Message)
rover1Parser.constructedMessageSignal.connect(server.sendClient1Message)
# Connect controller 0 signals
#rover0Controller.sendCommandSignal.connect(rover0Parser.constructMessage)
rover0Controller.sendCommandSignal.connect(rover0Command)
rover0Controller.positionChanged.connect(serverGui.setRoverIconPos)
rover0Controller.newRedBlockFound.connect(serverGui.moveRedBlock)
rover0Controller.newRedBlockFound.connect(rover1Controller.addRedBlock)
rover0Controller.newGreenBlockFound.connect(serverGui.moveGreenBlock)
rover0Controller.newGreenBlockFound.connect(rover1Controller.addGreenBlock)
rover0Controller.newBlueBlockFound.connect(serverGui.moveBlueBlock)
rover0Controller.newBlueBlockFound.connect(rover1Controller.addBlueBlock)
# Connect controller 1 signals
#rover1Controller.sendCommandSignal.connect(rover1Parser.constructMessage)
rover1Controller.sendCommandSignal.connect(rover1Command)
rover1Controller.positionChanged.connect(serverGui.setRoverIconPos)
rover1Controller.movedBlock.connect(serverGui.moveRedBlock)

# Show the GUI and start the server
serverGui.show()
server.startSelf.emit() # Will cause the server to run once Qt's event loop starts
exitCode = app.exec_()
sys.exit(exitCode)
