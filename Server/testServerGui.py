import sys
import unittest
from serverGui import MainWindow
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication

app = QApplication(sys.argv)

class serverGuiTest(unittest.TestCase):

    def setUp(self):
        self.window = MainWindow()

    def test_buttons(self):
        initialMapCount = self.window.mapClickCount
        QTest.mouseClick(self.window.mapButton, Qt.LeftButton)
        self.window.mapClickCount == initialMapCount + 1

        initialDiagnosticsCount = self.window.diagnosticsClickCount
        QTest.mouseClick(self.window.diagnosticsButton, Qt.LeftButton)
        self.assertEqual(self.window.diagnosticsClickCount, initialDiagnosticsCount + 1)

        initialResetCount = self.window.resetClickCount
        QTest.mouseClick(self.window.resetButton, Qt.LeftButton)
        self.assertEqual(self.window.resetClickCount, initialResetCount + 1)

        initialStartCount = self.window.startClickCount
        QTest.mouseClick(self.window.startButton, Qt.LeftButton)
        self.assertEqual(self.window.startClickCount, initialStartCount + 1)

        initialPauseCount = self.window.pauseClickCount
        QTest.mouseClick(self.window.pauseButton, Qt.LeftButton)
        self.assertEqual(self.window.pauseClickCount, initialPauseCount + 1)

if __name__ == "__main__":
    unittest.main()