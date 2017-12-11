import time

class RoverPositions:

    def __init__(self):
        self.rover0X = 0
        self.rover0Y = 0
        self.nextRover0X = 0
        self.nextRover0Y = 0
        self.rover1X =-1
        self.rover1Y = 0
        self.nextRover1X = -1
        self.nextRover1Y = 0

    def setRover0Pos(self, x, y):
        self.rover0X = x
        self.rover0Y = y

    def setNextRover0Pos(self, x, y):
        self.nextRover0X = x
        self.nextRover0Y = y

    def setRover1Pos(self, x, y):
        self.rover1X = x
        self.rover1Y = y

    def setNextRover1Pos(self, x, y):
        self.nextRover1X = x
        self.nextRover1Y = y

    def waitUntilRover0Clear(self):
        return
        '''while True:
            if self.rover1X != x and self.nextRover1X != x and \
                self.rover1Y != y and self.nextRover1Y != y:
                break
            time.sleep(0.1)'''

    def waitUntilRover1Clear(self):
        while True:
            '''if self.rover0X != x and self.nextRover0X != x and \
                self.rover0Y != y and self.nextRover0Y != y:
                break'''
            if self.rover0Y - self.rover1Y > 3 or (self.rover0X == 8 and self.rover0Y == 8):
                break
            time.sleep(0.5)