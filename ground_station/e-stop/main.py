from PySide2.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget
from PySide2.QtWidgets import QApplication, QMainWindow, QStatusBar
from PySide2.QtCore import QTimer
import subprocess
import os

e_stop_counter = 2

def excute_lock():
    global e_stop_counter 
    e_stop_counter += 1
    subprocess.Popen(['/home/autonobots/Desktop/not-a-boring/Autonobots_NAB_Competition/lock.sh'])

def execute_unlock():
    global e_stop_counter 
    e_stop_counter += 1
    subprocess.Popen(['/home/autonobots/Desktop/not-a-boring/Autonobots_NAB_Competition/unlock.sh'])


class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()

        # Create a status bar
        self.statusBar = QStatusBar()

        self.button_lock = QPushButton('e-stop - lock')
        self.button_lock.setFixedSize(300, 100)
        self.button_lock.clicked.connect(excute_lock)
        self.button_lock.setStyleSheet("background-color: red; font-size: 20px;")
      
        self.button_unlock = QPushButton('e-stop - unlock')
        self.button_unlock.setFixedSize(300, 100)
        self.button_unlock.clicked.connect(execute_unlock)
        self.button_unlock.setStyleSheet("background-color: green; font-size: 20px;")
        
        layout = QVBoxLayout()
        layout.addWidget(self.button_lock)
        layout.addWidget(self.button_unlock)
        layout.addWidget(self.statusBar)

        central_widget = QWidget()
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        
        # Set the status bar to the main window
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(500)
        # Set a message to the status bar
        self.statusBar.showMessage('Status: Ready')
    
    def update_status(self):
        host = '10.42.0.1'
        global e_stop_counter
        try:
            result = subprocess.run(['ping', '-c','1','-W','1',host], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
            if result.returncode == 0 and e_stop_counter % 2 == 0:
                print("hello")
                self.statusBar.showMessage('Status: Connected')
            else:
                self.statusBar.showMessage('Status: e-stop engaged')
        except Exception as e:
            self.statusBar.showMessage(f'Status: e-stop engaged')

app = QApplication([])

window = MainWindow()
window.show()

app.exec_()
