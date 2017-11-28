import socket
import threading
import time
from PyQt5.QtCore import *

class team13Server(QObject):

    maxBacklog = 5
    maxMessageSize = 1024

    # Signal emitted when a new message segment is received from client0
    client0Message = pyqtSignal(bytes)
    # Signal emitted when a new message segment is received from client1
    client1Message = pyqtSignal(bytes)
    # Signal emitted when the server stops running
    serverClosed = pyqtSignal()
    # Signal used to run the server when Qt's event loop starts
    startSelf = pyqtSignal()

    def __init__(self, host, port):
        super().__init__()
        self.hostname = host #'localhost'
        self.portNumber = port #50000
        self.client0 = None
        self.client1 = None
        self.sckt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Construct socket object
        self.clientsConnected = 0

    def runServer(self):

        self.sckt.bind((self.hostname, self.portNumber))                  # Bind to port
        self.sckt.listen(self.maxBacklog)                                 # Listen for incomming connections
        '''self.client0, address0 = self.sckt.accept()                       # Accept client connection
        self.clientsConnected = 1
        print('Client 0 connected')'''
        client0Thread = threading.Thread(target=self.monitorClient0)
        client0Thread.start()                                             # Start thread to monitor client 0
        '''self.client1, address1 = self.sckt.accept()
        self.clientsConnected = 2
        print('Client 1 connected')'''
        client1Thread = threading.Thread(target=self.monitorClient1)
        client1Thread.start()


        '''
        while True:
            try:
                receivedData = self.client0.recv(self.maxMessageSize)
            except socket.error:
                break
            if not receivedData:
                break
            self.client0Message.emit(receivedData)
            try:
                receivedData = self.client1.recv(self.maxMessageSize)
            except socket.error:
                break
            if not receivedData:
                break
            self.client1Message.emit(receivedData)
        self.serverClosed.emit()
        self.sckt.close()'''

    def monitorClient0(self):
        self.client0, address0 = self.sckt.accept()  # Accept client connection
        self.clientsConnected += 1
        print('Client 0 connected')

        while True:
            try:
                receivedData = self.client0.recv(self.maxMessageSize)
            except socket.error:
                break
            if not receivedData:
                break
            self.client0Message.emit(receivedData)
        self.clientsConnected -= 1
        if self.clientsConnected == 0:
            self.serverClosed.emit()
            self.sckt.close()

    def monitorClient1(self):
        while self.clientsConnected < 1:    # Wait for client 0 to connect
            time.sleep(0.1)

        self.client1, address1 = self.sckt.accept()
        self.clientsConnected = 2
        print('Client 1 connected')

        while True:
            try:
                receivedData = self.client1.recv(self.maxMessageSize)
            except socket.error:
                break
            if not receivedData:
                break
            self.client1Message.emit(receivedData)
        self.clientsConnected -= 1
        if self.clientsConnected == 0:
            self.serverClosed.emit()
            self.sckt.close()


    def closeServer(self):
        if self.clientsConnected > 0:
            self.client0.close()
            if self.clientsConnected > 1:
                self.client1.close()
            self.sckt.close()

    def sendClient0Message(self, message):
        if self.clientsConnected > 0:
            stringBytes = bytearray([ord(x) for x in message])
            self.client0.send(stringBytes)
        else:
            print('Rover 0 not connected')

    def sendClient1Message(self, message):
        if self.clientsConnected > 1:
            stringBytes = bytearray([ord(x) for x in message])
            self.client1.send(stringBytes)
        else:
            print('Rover 1 not connected')


#runServer()
