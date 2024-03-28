from PySide2.QtWidgets import QApplication, QPushButton
import subprocess


def excute_script():
    subprocess.Popen(['python', 'script.py'])

app = QApplication([])  

button = QPushButton('e-stop')
button.setFixedSize(300, 100)
button.clicked.connect(excute_script)
button.setStyleSheet("background-color: red; font-size: 20px;")
button.show()

app.exec_()