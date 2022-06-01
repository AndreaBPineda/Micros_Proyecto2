import sys
from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication

class App(QMainWindow):
	def __init__(self):
		super().__init__()
		uic.loadUi("INT_PRO.ui", self) # Ingresar nombre de su archivo .ui
		self.servo1.valueChanged.connect(self.get_value_servo1)
		self.servo2.valueChanged.connect(self.get_value_servo2)
		self.servo3.valueChanged.connect(self.get_value_servo3)
		self.servo4.valueChanged.connect(self.get_value_servo4)

	def get_value_servo1(self): # para el dial
		self.lbl_servo1.setText(str(self.servo1.value()))
		pass

	def get_value_servo2(self): # para el slider
		self.lbl_servo2.setText(str(self.servo2.value()))
		pass

	def get_value_servo3(self): # para el slider
		self.lbl_servo3.setText(str(self.servo3.value()))
		pass

	def get_value_servo4(self): # para el slider
		self.lbl_servo4.setText(str(self.servo4.value()))
		pass

if __name__ == '__main__':
	app = QApplication(sys.argv)
	GUI = App()
	GUI.show()
	sys.exit(app.exec_())

	#implementando 4 servos motores
	#se agregan más definiciones para el incluir los movimientos de la garra, antebrazo, brazo y superficie
	#se debe especificar cual es el límite del movimiento de los servos