import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import * # QApplication, QWidget, QMainWindow, QPushButton, QHBoxLayout, QVBoxLayout, QGraphicsScene, QGraphicsView
import commandDefs

class MainWindow(QWidget):

    # Signal to send command
    #                       rvr  cmd
    guiCommand = pyqtSignal(str, str)

    # Signal for start button
    startSignal = pyqtSignal()
    # Signal for pause button
    pauseSignal = pyqtSignal()
    # Signal for reset button
    resetSignal = pyqtSignal()
    # Signal for GUI close
    windowClosed = pyqtSignal()


    def __init__(self):
        super().__init__()

        # Stats for testing
        self.mapClickCount = 0
        self.diagnosticsClickCount = 0
        self.resetClickCount = 0
        self.startClickCount = 0
        self.pauseClickCount = 0

        # Declare buttons
        self.declareRightPanelButtons()

        # Create layout for buttons
        rightButtonsLayout = self.createRightButtonsLayout()

        # Create map widget
        self.createMapView()
        # Create diagnostics widget
        self.createDiagnosticsView()

        # Create stacked map and diagnostics widget
        self.leftDisplayWidget = QStackedWidget()
        self.leftDisplayWidget.addWidget(self.mapDisplayView)
        self.leftDisplayWidget.addWidget(self.diagnosticsWidget)
        self.leftDisplayWidget.setCurrentWidget(self.mapDisplayView)

        # Main layout holding display and buttons
        masterLayout = QHBoxLayout()
        masterLayout.addWidget(self.leftDisplayWidget)
        masterLayout.addLayout(rightButtonsLayout)

        self.setLayout(masterLayout)

        self.setGeometry(50, 50, 640, 480)
        self.setWindowTitle('Team 13 GUI')
        self.setStyleSheet("background-color: black")
        #self.show()

        self.setRoverIconPos(1, -1, 0)

    def declareRightPanelButtons(self):
        # Declare top buttons
        self.mapButton = QPushButton('Map')
        self.mapButton.clicked.connect(self.mapButtonHandler)
        self.mapButton.setStyleSheet("background-color: white")

        self.diagnosticsButton = QPushButton('Diagnostics')
        self.diagnosticsButton.clicked.connect(self.diagnosticsButtonHandler)
        self.diagnosticsButton.setStyleSheet("background-color: grey")

        # Declare bottom buttons
        self.resetButton = QPushButton('Reset')
        self.resetButton.clicked.connect(self.resetButtonHandler)
        self.resetButton.setStyleSheet("background-color: yellow")

        self.startButton = QPushButton('Start')
        self.startButton.clicked.connect(self.startButtonHandler)
        self.startButton.setStyleSheet("background-color: green")

        self.pauseButton = QPushButton('Pause')
        self.pauseButton.clicked.connect(self.pauseButtonHandler)
        self.pauseButton.setStyleSheet("background-color: red")

    def createRightButtonsLayout(self):
        topButtonsLayout = QVBoxLayout()
        topButtonsLayout.addWidget(self.mapButton)
        topButtonsLayout.addWidget(self.diagnosticsButton)
        topButtonsLayout.addStretch(1)

        bottomButtonsLayout = QVBoxLayout()
        bottomButtonsLayout.addStretch(1)
        bottomButtonsLayout.addWidget(self.resetButton)
        bottomButtonsLayout.addWidget(self.startButton)
        bottomButtonsLayout.addWidget(self.pauseButton)

        # Layout for all buttons on the right side
        allButtonsLayout = QVBoxLayout()
        allButtonsLayout.addLayout(topButtonsLayout)
        allButtonsLayout.addLayout(bottomButtonsLayout)

        return allButtonsLayout

    def createMapView(self):
        self.mapDisplayScene = QGraphicsScene(0, 0, 400, 400)

        # Add black grid to map
        for x in range(0,9):
            self.mapDisplayScene.addRect(20+40*x, 20, 4, 324, brush=QBrush(Qt.black))
        for y in range(0,9):
            self.mapDisplayScene.addRect(20, 20+40*y, 324, 4, brush=QBrush(Qt.black))

        # Add rovers in starting positions
        self.scoutRoverIcon = QGraphicsRectItem(14, 20+40*8-6, 16, 16)
        self.scoutRoverIcon.setBrush(QBrush(QColor(102,0,0))) # Maroon
        self.mapDisplayScene.addItem(self.scoutRoverIcon)
        self.moveRoverIcon = QGraphicsRectItem(14, 20+40*8-6, 16, 16)#-26, 20+40*8-6, 16, 16)
        self.moveRoverIcon.setBrush(QBrush(QColor(255, 102, 0)))
        self.mapDisplayScene.addItem(self.moveRoverIcon)

        # Add invisible boxes to be shown once discovered
        self.declareBoxes()

        self.mapDisplayView = QGraphicsView(self.mapDisplayScene)
        self.mapDisplayView.setStyleSheet("background-color: white")

    def declareBoxes(self):
        # Add sets of 3 each red, green, and blue boxes at 0,0
        self.redBoxes = [QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16),
                           QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16),
                           QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16)]
        for box in self.redBoxes:
            box.setBrush(QBrush(QColor(Qt.red)))
            box.setVisible(False)
            self.mapDisplayScene.addItem(box)

        self.greenBoxes = [QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16),
                           QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16),
                           QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16)]
        for box in self.greenBoxes:
            box.setBrush(QBrush(QColor(Qt.green)))
            box.setVisible(False)
            self.mapDisplayScene.addItem(box)

        self.blueBoxes = [QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16),
                           QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16),
                           QGraphicsRectItem(14, 20 + 40 * 8 - 6, 16, 16)]
        for box in self.blueBoxes:
            box.setBrush(QBrush(QColor(Qt.blue)))
            box.setVisible(False)
            self.mapDisplayScene.addItem(box)

    def createDiagnosticsView(self):
        # QLabel to display text
        self.diagnosticsLabel = QLabel("It works.")
        self.diagnosticsLabel.setStyleSheet("color: white")

        # Command builder
        self.roverSelector = QComboBox()
        self.roverSelector.setStyleSheet("background-color: white")
        for key in commandDefs.roverIDs:
            self.roverSelector.addItem(key)
        self.roverSelector.currentIndexChanged.connect(self.commandBuilderSelectionChanged)
        self.commandSelector = QComboBox()
        self.commandSelector.setStyleSheet("background-color: white")
        for key in commandDefs.commands:
            self.commandSelector.addItem(key)
        self.commandSelector.currentIndexChanged.connect(self.commandBuilderSelectionChanged)
        self.specifierSelector = QComboBox()
        self.specifierSelector.setStyleSheet("background-color: white")
        self.specifierSelector.addItem("")
        for key in commandDefs.specifiers:
            self.specifierSelector.addItem(key)
        self.specifierSelector.currentIndexChanged.connect(self.commandBuilderSelectionChanged)
        self.sendCommandButton = QPushButton("Send")
        self.sendCommandButton.setStyleSheet("background-color: white")
        self.sendCommandButton.setEnabled(False)
        self.sendCommandButton.clicked.connect(self.sendCommandButtonHandler)
        commandBuilderLayout = QHBoxLayout()
        commandBuilderLayout.addWidget(self.roverSelector)
        commandBuilderLayout.addWidget(self.commandSelector)
        commandBuilderLayout.addWidget(self.specifierSelector)
        commandBuilderLayout.addWidget(self.sendCommandButton)

        diagnosticsLayout = QVBoxLayout()
        diagnosticsLayout.addWidget(self.diagnosticsLabel)
        diagnosticsLayout.addLayout(commandBuilderLayout)
        self.diagnosticsWidget = QWidget()
        self.diagnosticsWidget.setLayout(diagnosticsLayout)

    def isCommandBuilderValid(self):
        if '0' in self.roverSelector.currentText():  # Rover 0, scout
            if "MOVE" in self.commandSelector.currentText():
                return "MOVE" in self.specifierSelector.currentText()
            elif "READ" in self.commandSelector.currentText():
                return len(self.specifierSelector.currentText()) == 0
            elif "PICKUP" in self.commandSelector.currentText():
                return False
            elif "STREAM" in self.commandSelector.currentText():
                return "SENSOR" in self.specifierSelector.currentText()
        else:   # Rover 1, mover
            if "MOVE" in self.commandSelector.currentText():
                return "MOVE" in self.specifierSelector.currentText()
            elif "READ" in self.commandSelector.currentText():
                return False
            elif "PICKUP" in self.commandSelector.currentText():
                return 'MOVE' in self.specifierSelector.currentText()
            elif "STREAM" in self.commandSelector.currentText():
                return "SENSOR" in self.specifierSelector.currentText()

    def mapButtonHandler(self, bool):
        self.mapClickCount += 1

        # Change button colors
        self.mapButton.setStyleSheet("background-color: white")
        self.diagnosticsButton.setStyleSheet("background-color: grey")

        # Swap widgets
        self.leftDisplayWidget.setCurrentWidget(self.mapDisplayView)

        print('Map button clicked')

    def diagnosticsButtonHandler(self, bool):
        self.diagnosticsClickCount += 1

        # Change button colors
        self.mapButton.setStyleSheet("background-color: grey")
        self.diagnosticsButton.setStyleSheet("background-color: white")

        # Swap widgets
        self.leftDisplayWidget.setCurrentWidget(self.diagnosticsWidget)

        print('Diagnostics button clicked')

    def resetButtonHandler(self, bool):
        self.resetClickCount += 1
        self.resetSignal.emit()
        print('Reset button clicked')

    def startButtonHandler(self, bool):
        self.startClickCount += 1
        self.startSignal.emit()
        print('Start button clicked')

    def pauseButtonHandler(self, bool):
        self.pauseClickCount += 1
        self.pauseSignal.emit()
        print('Pause button clicked')

    def sendCommandButtonHandler(self, bool):
        print('Send command button clicked')
        commandString = commandDefs.commands[self.commandSelector.currentText()] + ' ' + commandDefs.specifiers[self.specifierSelector.currentText()]
        self.guiCommand.emit(self.roverSelector.currentText(), commandString)

    def commandBuilderSelectionChanged(self, index):
        self.sendCommandButton.setEnabled(self.isCommandBuilderValid())

    def setRoverIconPos(self, roverID, newX, newY):
        if roverID == 0:
            self.scoutRoverIcon.setPos(40*newX, -40*newY)
        else:
            self.moveRoverIcon.setPos(40*newX,-40*newY)

    def moveRedBlock(self, blockIndex, newX, newY):
        self.redBoxes[blockIndex].setPos(40*newX, -40*newY)
        self.redBoxes[blockIndex].setVisible(True)

    def moveGreenBlock(self, blockIndex, newX, newY):
        self.greenBoxes[blockIndex].setPos(40*newX, -40*newY)
        self.greenBoxes[blockIndex].setVisible(True)

    def moveBlueBlock(self, blockIndex, newX, newY):
        self.blueBoxes[blockIndex].setPos(40*newX, -40*newY)
        self.blueBoxes[blockIndex].setVisible(True)

    def closeEvent(self, QCloseEvent):
        self.windowClosed.emit()

