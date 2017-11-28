import socket
from PyQt5.QtCore import *

class TestClient(QObject):

    hostname = 'localhost'
    portNumber = 50000
    maxMessageSize = 1024

    # Signals
    messageFromServer = pyqtSignal(bytes)
    begin = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # Construct socket object
        self.begin.connect(self.runTestClient)

    def sendMessageToServer(self, message):
        stringBytes = bytearray([ord(x) for x in message])
        self.sckt.send(stringBytes)

    def runTestClient(self):
        self.sckt.connect((self.hostname, self.portNumber))  # Connect to server

        print("Listening for messages...")
        while True:
            try:
                messageChunk = self.sckt.recv(self.maxMessageSize)
            except socket.error:
                exit()
            if len(messageChunk) == 0:
                exit()
            self.messageFromServer.emit(messageChunk)