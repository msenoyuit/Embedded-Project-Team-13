from serverGui import MainWindow
from PyQt5.QtWidgets import QApplication
import sys

app = QApplication(sys.argv)
Gui = MainWindow()
sys.exit(app.exec_())